# Add secure executable
add_executable(hello_trustzone_s secure.c)
target_link_libraries(hello_trustzone_s
        pico_stdlib
        hardware_watchdog
        pico_secure
)
pico_set_modified_binary_type(hello_trustzone_s sram
        RAM 0x20070000 60k
        SCRATCH_X 0x2007f000 2k
        SCRATCH_Y 0x2007f800 2k)
pico_package_uf2_output(hello_trustzone_s 0x10000000)
pico_add_extra_outputs(hello_trustzone_s)

# Add non-secure executable
add_executable(hello_trustzone_ns nonsecure.c)
pico_set_uf2_family(hello_trustzone_ns "rp2350-arm-ns")
target_link_libraries(hello_trustzone_ns
        pico_stdlib
)
pico_enable_stdio_uart(hello_trustzone_ns 0)
pico_enable_stdio_usb(hello_trustzone_ns 1)
pico_set_modified_binary_type(hello_trustzone_ns flash
        RAM 0x20000000 448k)
pico_add_extra_outputs(hello_trustzone_ns)

# Set security options
pico_set_security_options(hello_trustzone_s hello_trustzone_ns
    USER_IRQ USB
    NONSECURE_TIMER 1
)

# Create combined UF2
pico_concatenate_uf2_outputs(hello_trustzone hello_trustzone_ns hello_trustzone_s)
