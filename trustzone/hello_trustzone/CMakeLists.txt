# Add secure executable
add_executable(hello_trustzone_s secure.c)
target_link_libraries(hello_trustzone_s
        pico_stdlib
        hardware_watchdog
        pico_secure
)

pico_set_binary_type(hello_trustzone_s no_flash)
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
pico_add_extra_outputs(hello_trustzone_ns)

# Set security options
pico_set_security_options(hello_trustzone_s hello_trustzone_ns
    USER_IRQ USB
    NONSECURE_TIMER 1
)

# Set security ram split
pico_set_security_ram_split(hello_trustzone_s hello_trustzone_ns SIMPLE 64k 2k)

# Create combined UF2 (ns first)
pico_concatenate_uf2_outputs(hello_trustzone hello_trustzone_ns hello_trustzone_s)



# no_flash example
# Add secure executable
add_executable(hello_trustzone_no_flash_s secure.c)
target_link_libraries(hello_trustzone_no_flash_s
        pico_stdlib
        hardware_watchdog
        pico_secure
)

pico_set_binary_type(hello_trustzone_no_flash_s no_flash)
pico_add_extra_outputs(hello_trustzone_no_flash_s)

# Add non-secure executable
add_executable(hello_trustzone_no_flash_ns nonsecure.c)
pico_set_uf2_family(hello_trustzone_no_flash_ns "rp2350-arm-ns")
target_link_libraries(hello_trustzone_no_flash_ns
        pico_stdlib
)
pico_enable_stdio_uart(hello_trustzone_no_flash_ns 0)
pico_enable_stdio_usb(hello_trustzone_no_flash_ns 1)

pico_set_binary_type(hello_trustzone_no_flash_ns no_flash)
pico_add_extra_outputs(hello_trustzone_no_flash_ns)

# Set security options
pico_set_security_options(hello_trustzone_no_flash_s hello_trustzone_no_flash_ns
    USER_IRQ USB
    NONSECURE_TIMER 1
)

# Set security ram split (with NO_FLASH)
pico_set_security_ram_split(hello_trustzone_no_flash_s hello_trustzone_no_flash_ns SIMPLE 64k 2k NO_FLASH)

# Create combined CMake target
add_custom_target(hello_trustzone_no_flash ALL
        DEPENDS hello_trustzone_no_flash_s hello_trustzone_no_flash_ns)
