if(CONFIG_ZMK_KSCAN_74HC164_MATRIX_DRIVER)
    zephyr_library()
    zephyr_library_sources(src/kscan_74hc164_matrix.c)
endif()
