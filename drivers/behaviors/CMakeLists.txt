if(CONFIG_ZMK_BEHAVIOR_INERTIA)
  zephyr_library_amend()
  zephyr_library_sources(behavior_inertia.c)
endif()
