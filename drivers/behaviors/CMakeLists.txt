if(CONFIG_ZMK_BEHAVIOR_INERTIA)
  zephyr_library_named(zmk_behavior_inertia)
  zephyr_library_sources(behavior_inertia.c)
endif()
