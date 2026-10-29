/*
 * SPDX-License-Identifier: MIT
 *
 * Crash guard. Instead of halting forever on a fatal error (the Zephyr default,
 * which leaves a split central dead until it is power-cycled), this:
 *   - overrides k_sys_fatal_error_handler() to record the reason, faulting
 *     thread and PC/LR in RAM that survives a soft reset, then reboots;
 *   - watches work queues with the task watchdog so a queue that stops running
 *     (deadlock or a thread hogging the CPU) is recorded and rebooted too, with
 *     the hardware watchdog as a fallback when interrupts are blocked;
 *   - persists the last record and a crash counter to settings so it can be
 *     reported later through zmk_crash_guard_format().
 */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/fatal.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/linker/section_tags.h>
#include <zephyr/logging/log.h>
#include <zephyr/logging/log_ctrl.h>
#include <zephyr/sys/reboot.h>

#if IS_ENABLED(CONFIG_HWINFO)
#include <zephyr/drivers/hwinfo.h>
#endif

#if IS_ENABLED(CONFIG_ZMK_CRASH_GUARD_WATCHDOG)
#include <zephyr/task_wdt/task_wdt.h>
#endif

#if IS_ENABLED(CONFIG_SETTINGS)
#include <zephyr/settings/settings.h>
#endif

#if defined(CONFIG_CPU_CORTEX_M)
#include <cmsis_core.h>
#endif

#include <zmk_crash_guard.h>

LOG_MODULE_REGISTER(crash_guard, LOG_LEVEL_INF);

#define CG_MAGIC 0x43524153u
#define CG_NAME_LEN 24
#define CG_STATE_LEN 16

enum cg_kind {
    CG_KIND_NONE = 0,
    CG_KIND_FATAL,
    CG_KIND_HANG,
    CG_KIND_HW_WATCHDOG,
    CG_KIND_LOCKUP,
};

struct cg_info {
    uint32_t kind;
    uint32_t reason;
    uint32_t pc;
    uint32_t lr;
    uint32_t cfsr;
    uint32_t fault_addr;
    uint32_t exc;
    uint32_t wait_obj;
    uint32_t uptime_s;
    char thread[CG_NAME_LEN];
    char label[CG_NAME_LEN];
    char state[CG_STATE_LEN];
};

BUILD_ASSERT(sizeof(struct cg_info) % sizeof(uint32_t) == 0, "cg_info must be word sized");

struct cg_retained {
    uint32_t magic;
    struct cg_info info;
    uint32_t check;
};

struct cg_persist {
    uint32_t count;
    struct cg_info info;
};

// Not cleared at startup, so a record written right before a soft reset is
// still readable on the next boot. Prefer a RAM block that is excluded from
// Zephyr's RAM (chosen `zmk,crash-guard-ram`): a bootloader that runs between
// the reset and the application (e.g. the Adafruit nRF52 bootloader, which
// uses RAM from 0x20008000) may overwrite an ordinary __noinit variable.
#if DT_HAS_CHOSEN(zmk_crash_guard_ram)
BUILD_ASSERT(DT_REG_SIZE(DT_CHOSEN(zmk_crash_guard_ram)) >= sizeof(struct cg_retained),
             "zmk,crash-guard-ram is too small");
#define CG_RETAINED ((struct cg_retained *)DT_REG_ADDR(DT_CHOSEN(zmk_crash_guard_ram)))
#else
static __noinit struct cg_retained cg_retained_ram;
#define CG_RETAINED (&cg_retained_ram)
#endif

static struct cg_info boot_new;
static bool boot_has_new;
static uint32_t boot_cause;
static struct cg_persist last;
static bool last_valid;

static uint32_t cg_checksum(const struct cg_info *info) {
    const uint32_t *words = (const uint32_t *)info;
    uint32_t sum = CG_MAGIC;

    for (size_t i = 0; i < sizeof(*info) / sizeof(uint32_t); i++) {
        sum = ((sum << 5) | (sum >> 27)) ^ words[i];
    }
    return sum;
}

static void cg_copy(char *dst, size_t size, const char *src) {
    size_t i = 0;

    if (src != NULL) {
        for (; i + 1 < size && src[i] != '\0'; i++) {
            dst[i] = src[i];
        }
    }
    dst[i] = '\0';
}

static void cg_thread_name(char *dst, size_t size, k_tid_t thread) {
    const char *name = thread != NULL ? k_thread_name_get(thread) : NULL;

    if (name != NULL && name[0] != '\0') {
        cg_copy(dst, size, name);
    } else {
        snprintf(dst, size, "thread@%08x", (unsigned int)(uintptr_t)thread);
    }
}

static FUNC_NORETURN void cg_store_and_reboot(const struct cg_info *info) {
    struct cg_retained *retained = CG_RETAINED;

    retained->info = *info;
    retained->check = cg_checksum(&retained->info);
    retained->magic = CG_MAGIC;

    LOG_PANIC();
    sys_reboot(SYS_REBOOT_COLD);
}

void k_sys_fatal_error_handler(unsigned int reason, const struct arch_esf *esf) {
    struct cg_info info;

    memset(&info, 0, sizeof(info));
    info.kind = CG_KIND_FATAL;
    info.reason = reason;
    info.uptime_s = (uint32_t)(k_uptime_get() / MSEC_PER_SEC);
    cg_thread_name(info.thread, sizeof(info.thread), k_current_get());

#if defined(CONFIG_CPU_CORTEX_M)
    if (esf != NULL) {
        info.pc = esf->basic.pc;
        info.lr = esf->basic.lr;
        // IPSR of the faulting context: 0 = thread mode, >= 16 = IRQ (n - 16).
        info.exc = esf->basic.xpsr & 0x1FFu;
    }
    info.cfsr = SCB->CFSR;
    if (info.cfsr & SCB_CFSR_MMARVALID_Msk) {
        info.fault_addr = SCB->MMFAR;
    } else if (info.cfsr & SCB_CFSR_BFARVALID_Msk) {
        info.fault_addr = SCB->BFAR;
    }
#else
    ARG_UNUSED(esf);
#endif

    cg_store_and_reboot(&info);
}

#if IS_ENABLED(CONFIG_ZMK_CRASH_GUARD_WATCHDOG)

struct cg_watch {
    struct k_work_delayable work;
    struct k_work_q *queue;
    const char *label;
    int channel;
};

static struct cg_watch watches[CONFIG_ZMK_CRASH_GUARD_MAX_WATCHES];
static size_t watch_count;
static bool wdt_started;

// Runs in the kernel timer ISR when a watched queue missed its deadline.
static void cg_hang_callback(int channel_id, void *user_data) {
    struct cg_watch *watch = user_data;
    struct cg_info info;
    k_tid_t running = k_current_get();
    k_tid_t watched = k_work_queue_thread_get(watch->queue);

    memset(&info, 0, sizeof(info));
    info.kind = CG_KIND_HANG;
    info.reason = (uint32_t)channel_id;
    info.uptime_s = (uint32_t)(k_uptime_get() / MSEC_PER_SEC);
    cg_copy(info.label, sizeof(info.label), watch->label);
    cg_thread_name(info.thread, sizeof(info.thread), running);

    if (watched != NULL) {
        k_thread_state_str(watched, info.state, sizeof(info.state));
        // Address of the wait queue it is blocked on; it lies inside the
        // msgq/sem/mutex object, so it can be resolved with the ELF symbols.
        info.wait_obj = (uint32_t)(uintptr_t)watched->base.pended_on;
    }

#if defined(CONFIG_CPU_CORTEX_M) && defined(CONFIG_THREAD_STACK_INFO)
    // PC/LR where the interrupted thread was running, taken from the exception
    // frame stacked on its PSP (only trusted when the PSP is inside its stack).
    if (running != NULL) {
        uintptr_t psp = __get_PSP();
        uintptr_t start = running->stack_info.start;
        uintptr_t end = start + running->stack_info.size;

        if (psp >= start && psp + 8 * sizeof(uint32_t) <= end) {
            const uint32_t *frame = (const uint32_t *)psp;

            info.lr = frame[5];
            info.pc = frame[6];
        }
    }
#endif

    cg_store_and_reboot(&info);
}

static void cg_feed_handler(struct k_work *work) {
    struct k_work_delayable *dwork = k_work_delayable_from_work(work);
    struct cg_watch *watch = CONTAINER_OF(dwork, struct cg_watch, work);

    task_wdt_feed(watch->channel);
    k_work_schedule_for_queue(watch->queue, &watch->work,
                              K_MSEC(CONFIG_ZMK_CRASH_GUARD_WATCHDOG_FEED_MS));
}

static int cg_arm_watch(struct cg_watch *watch) {
    int channel =
        task_wdt_add(CONFIG_ZMK_CRASH_GUARD_WATCHDOG_TIMEOUT_MS, cg_hang_callback, watch);

    if (channel < 0) {
        LOG_ERR("Failed to add watchdog channel for %s: %d", watch->label, channel);
        return channel;
    }
    watch->channel = channel;
    k_work_schedule_for_queue(watch->queue, &watch->work, K_NO_WAIT);
    return 0;
}

int zmk_crash_guard_watch_work_q(struct k_work_q *queue, const char *label) {
    if (watch_count >= ARRAY_SIZE(watches)) {
        LOG_ERR("No free watch slot for %s", label);
        return -ENOMEM;
    }

    struct cg_watch *watch = &watches[watch_count++];

    watch->queue = queue;
    watch->label = label;
    watch->channel = -1;
    k_work_init_delayable(&watch->work, cg_feed_handler);

    return wdt_started ? cg_arm_watch(watch) : 0;
}

static void cg_start_watchdog(void) {
    const struct device *hw_wdt = NULL;
    int err;

#if IS_ENABLED(CONFIG_ZMK_CRASH_GUARD_HW_WATCHDOG) && DT_NODE_HAS_STATUS_OKAY(DT_ALIAS(watchdog0))
    hw_wdt = DEVICE_DT_GET(DT_ALIAS(watchdog0));
    if (!device_is_ready(hw_wdt)) {
        LOG_WRN("Hardware watchdog not ready, using the task watchdog only");
        hw_wdt = NULL;
    }
#endif

    err = task_wdt_init(hw_wdt);
    if (err != 0 && hw_wdt != NULL) {
        LOG_WRN("Hardware watchdog fallback failed (%d), using the task watchdog only", err);
        err = task_wdt_init(NULL);
    }
    if (err != 0) {
        LOG_ERR("Task watchdog init failed: %d", err);
        return;
    }

    wdt_started = true;
    for (size_t i = 0; i < watch_count; i++) {
        cg_arm_watch(&watches[i]);
    }
    zmk_crash_guard_watch_work_q(&k_sys_work_q, "sysworkq");
}

#else

int zmk_crash_guard_watch_work_q(struct k_work_q *queue, const char *label) {
    ARG_UNUSED(queue);
    ARG_UNUSED(label);
    return -ENOTSUP;
}

#endif // CONFIG_ZMK_CRASH_GUARD_WATCHDOG

#if IS_ENABLED(CONFIG_SETTINGS)

static int cg_settings_set(const char *name, size_t len, settings_read_cb read_cb, void *cb_arg) {
    struct cg_persist stored;
    int rc;

    if (!settings_name_steq(name, "last", NULL)) {
        return -ENOENT;
    }
    if (len != sizeof(stored)) {
        return -EINVAL;
    }
    rc = read_cb(cb_arg, &stored, sizeof(stored));
    if (rc < 0) {
        return rc;
    }
    last = stored;
    return 0;
}

SETTINGS_STATIC_HANDLER_DEFINE(crash_guard, "crash_guard", NULL, cg_settings_set, NULL, NULL);

static void cg_persist_work_handler(struct k_work *work) {
    ARG_UNUSED(work);

    settings_subsys_init();
    settings_load_subtree("crash_guard");

    if (boot_has_new) {
        last.count++;
        last.info = boot_new;
        int rc = settings_save_one("crash_guard/last", &last, sizeof(last));
        if (rc != 0) {
            LOG_WRN("Failed to save the crash record: %d", rc);
        }
    }
    last_valid = true;
}

static K_WORK_DELAYABLE_DEFINE(cg_persist_work, cg_persist_work_handler);

#endif // CONFIG_SETTINGS

static int cg_init(void) {
    struct cg_retained *retained = CG_RETAINED;

    if (retained->magic == CG_MAGIC && retained->check == cg_checksum(&retained->info)) {
        boot_new = retained->info;
        boot_has_new = true;
    }
    retained->magic = 0;

#if IS_ENABLED(CONFIG_HWINFO)
    uint32_t cause = 0;

    if (hwinfo_get_reset_cause(&cause) == 0) {
        boot_cause = cause;
        hwinfo_clear_reset_cause();
    }

    // The hardware watchdog / lockup resets the chip without running our
    // handlers, so only the reset cause tells us it happened.
    if (!boot_has_new && (boot_cause & (RESET_WATCHDOG | RESET_CPU_LOCKUP))) {
        memset(&boot_new, 0, sizeof(boot_new));
        boot_new.kind = (boot_cause & RESET_WATCHDOG) ? CG_KIND_HW_WATCHDOG : CG_KIND_LOCKUP;
        boot_has_new = true;
    }
#endif

    if (boot_has_new) {
        LOG_WRN("Recovered from a crash: kind %u reason %u thread %s pc 0x%08x lr 0x%08x",
                boot_new.kind, boot_new.reason, boot_new.thread, boot_new.pc, boot_new.lr);
    }

#if IS_ENABLED(CONFIG_ZMK_CRASH_GUARD_WATCHDOG)
    cg_start_watchdog();
#endif

#if IS_ENABLED(CONFIG_SETTINGS)
    k_work_schedule(&cg_persist_work, K_SECONDS(2));
#else
    if (boot_has_new) {
        last.count = 1;
        last.info = boot_new;
    }
    last_valid = true;
#endif

    return 0;
}

SYS_INIT(cg_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

static void cg_appendf(char *buf, size_t size, int *off, const char *fmt, ...) {
    size_t pos = (size_t)*off;
    va_list ap;
    int n;

    va_start(ap, fmt);
    n = vsnprintf(pos < size ? buf + pos : NULL, pos < size ? size - pos : 0, fmt, ap);
    va_end(ap);
    if (n > 0) {
        *off += n;
    }
}

static const char *cg_boot_cause_str(uint32_t cause) {
#if IS_ENABLED(CONFIG_HWINFO)
    if (cause & RESET_WATCHDOG) {
        return "HW watchdog";
    }
    if (cause & RESET_CPU_LOCKUP) {
        return "CPU lockup";
    }
    if (cause & RESET_SOFTWARE) {
        return "Software reset";
    }
    if (cause & RESET_PIN) {
        return "Reset pin";
    }
    if (cause & RESET_LOW_POWER_WAKE) {
        return "Wake from sleep";
    }
    if (cause & RESET_BROWNOUT) {
        return "Brownout";
    }
    if (cause & RESET_DEBUG) {
        return "Debugger";
    }
#endif
    return cause == 0 ? "Power on" : "Other";
}

static const char *cg_fatal_reason_str(uint32_t reason) {
    switch (reason) {
    case K_ERR_CPU_EXCEPTION:
        return "CPU exception";
    case K_ERR_SPURIOUS_IRQ:
        return "Spurious IRQ";
    case K_ERR_STACK_CHK_FAIL:
        return "Stack overflow";
    case K_ERR_KERNEL_OOPS:
        return "Kernel oops";
    case K_ERR_KERNEL_PANIC:
        return "Kernel panic";
    default:
        return "Fatal error";
    }
}

int zmk_crash_guard_format(char *buf, size_t size) {
    int off = 0;
    struct cg_persist rec;

    if (last_valid) {
        rec = last;
    } else {
        // Settings not merged yet (first seconds after boot): show this boot's record only.
        memset(&rec, 0, sizeof(rec));
        if (boot_has_new) {
            rec.count = 1;
            rec.info = boot_new;
        }
    }
    const struct cg_info *info = &rec.info;

    cg_appendf(buf, size, &off, "[Boot] %s%s\n", cg_boot_cause_str(boot_cause),
               boot_has_new ? " (after crash)" : "");

    if (rec.count == 0 || info->kind == CG_KIND_NONE) {
        cg_appendf(buf, size, &off, "[Crash] none recorded\n");
        return off;
    }

    switch (info->kind) {
    case CG_KIND_FATAL:
        cg_appendf(buf, size, &off, "[Crash] #%u %s in %s | up:%us\n", rec.count,
                   cg_fatal_reason_str(info->reason), info->thread, info->uptime_s);
        cg_appendf(buf, size, &off, "  PC:0x%08x LR:0x%08x", info->pc, info->lr);
        if (info->exc >= 16) {
            cg_appendf(buf, size, &off, " IRQ:%u", info->exc - 16);
        } else if (info->exc != 0) {
            cg_appendf(buf, size, &off, " EXC:%u", info->exc);
        }
        if (info->reason == K_ERR_CPU_EXCEPTION || info->reason == K_ERR_STACK_CHK_FAIL) {
            cg_appendf(buf, size, &off, " CFSR:0x%08x", info->cfsr);
            if (info->fault_addr != 0) {
                cg_appendf(buf, size, &off, " ADDR:0x%08x", info->fault_addr);
            }
        }
        cg_appendf(buf, size, &off, "\n");
        break;
    case CG_KIND_HANG:
        cg_appendf(buf, size, &off, "[Crash] #%u Hang: %s %s wait:0x%08x | up:%us\n", rec.count,
                   info->label, info->state, info->wait_obj, info->uptime_s);
        cg_appendf(buf, size, &off, "  running %s PC:0x%08x LR:0x%08x\n", info->thread,
                   info->pc, info->lr);
        break;
    case CG_KIND_HW_WATCHDOG:
        cg_appendf(buf, size, &off, "[Crash] #%u HW watchdog reset (IRQs blocked, no details)\n",
                   rec.count);
        break;
    case CG_KIND_LOCKUP:
        cg_appendf(buf, size, &off, "[Crash] #%u CPU lockup reset (no details)\n", rec.count);
        break;
    default:
        cg_appendf(buf, size, &off, "[Crash] #%u unknown record\n", rec.count);
        break;
    }

    return off;
}
