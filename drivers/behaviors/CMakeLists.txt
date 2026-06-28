if(CONFIG_ZMK_BEHAVIOR_INERTIA)
  zephyr_library_named(zmk_behavior_inertia)
  # ZMK keeps app/include private to the `app` target, so an out-of-tree
  # behavior library must add it explicitly to find <zmk/behavior.h> etc.
  zephyr_library_include_directories(${APPLICATION_SOURCE_DIR}/include)
  zephyr_library_sources(behavior_inertia.c)
endif()
