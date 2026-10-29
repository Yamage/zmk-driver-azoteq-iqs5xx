# ZMK driver for Azoteq IQS5XX trackpads

## Compatibility

This driver should work with any IQS5XX based trackpad. However, it has only been tested and known to work with the following models:

- TPS43 (Reached EOL)
- TPS65 (Reached EOL) per @Mac10goesBRRRT

Feel free to send a pull request if you test with any of the following models:

- TPR40
- TPR48
- TPR54
- TPE60
- TPE48
- TPS48

## Supported features

- Trackpad movement.
- Single finger tap: Reported as a left click.
- Two finger tap: Reported as a right click.
- Press and hold: Reported as a continuos left click (allows click and drag).
- Vertical scroll.
- Horizontal scroll.

## Crash guard (optional)

`CONFIG_ZMK_CRASH_GUARD=y` replaces Zephyr's default fatal error handler, which
halts the MCU forever (a split central then stops both halves until it is
power-cycled), with one that records the failure and reboots:

- **Fatal errors** (CPU fault, stack overflow, kernel oops such as a Bluetooth
  assert): reason, faulting thread, PC/LR, CFSR and fault address.
- **Hangs** (`CONFIG_ZMK_CRASH_GUARD_WATCHDOG`, default on): the system work
  queue and the trackpad work queue feed a task watchdog. A queue that does not
  run for `CONFIG_ZMK_CRASH_GUARD_WATCHDOG_TIMEOUT_MS` (5 s) is recorded with its
  state, the object it waits on, and the thread/PC that was running.
- **Blocked interrupts**: the hardware watchdog backs the task watchdog
  (`CONFIG_ZMK_CRASH_GUARD_HW_WATCHDOG`); such a reset is detected from the
  reset cause, without details.

The record is kept in RAM that survives the soft reset, then saved to settings
with a crash counter. `zmk_crash_guard_format()` prints it; the
`INERTIA_BLE_DIAG` behavior action appends it to its report, e.g.:

```
[Boot] Software reset (after crash)
[Crash] #1 Stack overflow in iqs5xx | up:1832s
  PC:0x00054c94 LR:0x00054c01 CFSR:0x00000000
```

Resolve PC/LR (and a hang's `wait:` address) with the matching `zmk.elf`, e.g.
`arm-zephyr-eabi-addr2line -f -e zmk.elf 0x00054c94` or `arm-zephyr-eabi-nm -n`.

If a bootloader runs between the reset and the application, keep the record out
of its RAM: reserve a block outside Zephyr's RAM and point the chosen
`zmk,crash-guard-ram` node at it (otherwise a `__noinit` variable is used):

```
/ {
    chosen { zmk,crash-guard-ram = &crash_guard_ram; };
    crash_guard_ram: crash-guard-ram@2003c000 { reg = <0x2003c000 0x100>; };
};
&sram0 { reg = <0x20000000 DT_SIZE_K(240)>; };
```

## Usage

- Specify a node with the "azoteq,iqs5xx" compatible inside an i2c node in your keyboard overlay.
- Reference it from an input listener:

```
/ {
    tps43_input: tps43_input {
        compatible = "zmk,input-listener";
        device = <&tps43>;
    };
};

&arduino_i2c {
    status = "okay";
    tps43: iqs5xx@74 {
        status = "okay";
        compatible = "azoteq,iqs5xx";
        reg = <0x74>;

        reset-gpios = <&arduino_header 14 GPIO_ACTIVE_LOW>;
        rdy-gpios = <&arduino_header 15 GPIO_ACTIVE_HIGH>;

        /*
         * Potentially non-exhaustive list of configuration options.
         * See: dts/bindings/input/azoteq,iqs5xx-common.yaml for a full list.
         */
        one-finger-tap;
        press-and-hold;
        press-and-hold-time = <250>;
        two-finger-tap;

        scroll;
        natural-scroll-y;
        natural-scroll-x;

        bottom-beta = <5>;
        stationary-threshold = <5>;

        switch-xy;
    };
};
```

