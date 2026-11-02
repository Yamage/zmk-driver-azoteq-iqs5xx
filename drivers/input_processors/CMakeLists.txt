if(CONFIG_ZMK_INPUT_PROCESSOR_SCROLL_SPEED)
  zephyr_library_named(zmk_input_processor_scroll_speed)
  # ZMK keeps app/include private to the `app` target, so an out-of-tree
  # input processor must add it explicitly to find <drivers/input_processor.h> etc.
  zephyr_library_include_directories(${APPLICATION_SOURCE_DIR}/include)
  zephyr_library_sources(input_processor_scroll_speed.c)
endif()
