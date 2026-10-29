if(CONFIG_ZMK_CRASH_GUARD)
  zephyr_library_named(zmk_crash_guard)
  zephyr_library_sources(crash_guard.c)
endif()
