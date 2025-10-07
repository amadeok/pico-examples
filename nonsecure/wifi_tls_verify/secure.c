#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/watchdog.h"
#include "pico/secure.h"

#include "hardware/structs/sau.h"
#include "hardware/structs/scb.h"


bool repeating_timer_callback(__unused struct repeating_timer *t) {
    printf("Repeat at %lld\n", time_us_64());
    watchdog_update();

    return true;
}


void hardfault_callback(void) {
    if (m33_hw->dhcsr & M33_DHCSR_C_DEBUGEN_BITS) {
        __breakpoint();
    } else {
        rom_reset_usb_boot(0, 0);
    }
}


int main()
{
    int rc;

    stdio_init_all();

    if (watchdog_enable_caused_reboot()) {
        printf("This was a watchdog reboot - resetting\n");
        rom_reset_usb_boot(0, 0);
    }

    boot_info_t info;
    rom_get_boot_info(&info);
    printf("Boot partition: %d\n", info.partition);

    int ns_partition = rom_get_owned_partition(info.partition);
    printf("Matching Non-Secure partition: %d\n", ns_partition);
    rc = rom_roll_qmi_to_partition(ns_partition);
    printf("Rolled QMI to Non-Secure partition, rc=%d\n", rc);

    rc = rom_set_ns_api_permission(BOOTROM_NS_API_reboot, true);
    printf("Enable Reboot, rc=%d\n", rc);
    rc = rom_set_ns_api_permission(BOOTROM_NS_API_get_sys_info, true);
    printf("Enable Sys Info, rc=%d\n", rc);

    // Enable NS GPIO access to CYW43 GPIOs
    gpio_assign_to_ns(CYW43_DEFAULT_PIN_WL_REG_ON, true);
    gpio_assign_to_ns(CYW43_DEFAULT_PIN_WL_DATA_OUT, true);
    gpio_assign_to_ns(CYW43_DEFAULT_PIN_WL_DATA_IN, true);
    gpio_assign_to_ns(CYW43_DEFAULT_PIN_WL_HOST_WAKE, true);
    gpio_assign_to_ns(CYW43_DEFAULT_PIN_WL_CLOCK, true);
    gpio_assign_to_ns(CYW43_DEFAULT_PIN_WL_CS, true);

    // XIP is NS Code
    secure_sau_configure_region(0, XIP_BASE, XIP_END, true, false);
    // SRAM0-3 (up to 2003000) is NS data
    secure_sau_configure_region(1, SRAM_BASE, 0x20030000, true, false);
    // SRAM8-9 is NS scratch
    secure_sau_configure_region(2, SRAM8_BASE, SRAM_END, true, false);

    secure_sau_set_enabled(true);
    printf("SAU Configured & Enabled\n");

    secure_install_default_hardfault_handler(hardfault_callback);

    struct repeating_timer timer;
    watchdog_enable(2000, true);
    add_repeating_timer_ms(1000, repeating_timer_callback, NULL, &timer);

    secure_launch_nonsecure_binary(XIP_BASE + 0x200, SRAM8_BASE);

    printf("Shouldn't return from jump\n");
}
