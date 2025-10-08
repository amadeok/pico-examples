add_subdirectory_exclude_platforms(hello_trustzone rp2040 rp2350-riscv)

if (PICO_CYW43_SUPPORTED) # set by PICO_BOARD=pico_w
    add_subdirectory_exclude_platforms(wifi_tls_verify rp2040 rp2350-riscv)
endif()
