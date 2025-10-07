# Add executable. Default name is the project name, version 0.1
add_executable(tls_verify_secure secure.c)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(tls_verify_secure 1)
pico_enable_stdio_usb(tls_verify_secure 0)

# Add the standard library to the build
target_link_libraries(tls_verify_secure
        pico_stdlib
        pico_rand
        hardware_exception
        hardware_watchdog
        pico_secure
)

# Add the standard include files to the build
target_include_directories(tls_verify_secure PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_set_binary_type(tls_verify_secure no_flash)

pico_set_linker_script(tls_verify_secure ${CMAKE_CURRENT_LIST_DIR}/memmap_s.ld)

pico_sign_binary(tls_verify_secure ${CMAKE_CURRENT_LIST_DIR}/private.pem)

pico_package_uf2_output(tls_verify_secure 0x10000000)

pico_add_extra_outputs(tls_verify_secure)

# Add non-secure executable
add_executable(tls_verify_nonsecure nonsecure.c tls_common.c)

pico_set_uf2_family(tls_verify_nonsecure "rp2350-arm-ns")

pico_enable_stdio_uart(tls_verify_nonsecure 0)
pico_enable_stdio_usb(tls_verify_nonsecure 1)

target_link_libraries(tls_verify_nonsecure
        pico_stdlib
        pico_cyw43_arch_lwip_threadsafe_background
        pico_lwip_mbedtls
        pico_mbedtls
)

target_include_directories(tls_verify_nonsecure PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

target_compile_definitions(tls_verify_nonsecure PRIVATE
    WIFI_SSID=\"${WIFI_SSID}\"
    WIFI_PASSWORD=\"${WIFI_PASSWORD}\"

    # By default verification is optional (MBEDTLS_SSL_VERIFY_OPTIONAL)
    # Make it required for this test
    ALTCP_MBEDTLS_AUTHMODE=MBEDTLS_SSL_VERIFY_REQUIRED
)

pico_set_linker_script(tls_verify_nonsecure ${CMAKE_CURRENT_LIST_DIR}/memmap_ns.ld)

pico_set_float_implementation(tls_verify_nonsecure none)

pico_add_extra_outputs(tls_verify_nonsecure)


pico_set_security_defines(tls_verify_secure tls_verify_nonsecure
    ALLOW_NONSECURE_STDIO
    ALLOW_NONSECURE_RAND
    ALLOW_NONSECURE_DMA
    ALLOW_NONSECURE_USER_IRQ
    ALLOW_NONSECURE_PIO
    ALLOW_NONSECURE_GPIO
    ALLOW_NONSECURE_USB
    ASSIGN_NONSECURE_TIMER 1
)


# Create combined UF2
add_custom_target(tls_verify_combined_uf2 ALL
    ${CMAKE_COMMAND} -E cat
        ${CMAKE_CURRENT_BINARY_DIR}/tls_verify_nonsecure.uf2
        ${CMAKE_CURRENT_BINARY_DIR}/tls_verify_secure.uf2
        > ${CMAKE_CURRENT_BINARY_DIR}/tls_verify_combined.uf2)

add_dependencies(tls_verify_combined_uf2 tls_verify_secure tls_verify_nonsecure)
