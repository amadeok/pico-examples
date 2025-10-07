#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "boot/picobin.h"
#include "hardware/watchdog.h"
#include "pico/secure.h"

#include "hardware/structs/sau.h"
#include "hardware/structs/qmi.h"
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
    stdio_init_all();

    printf("Lets go\n");

    if (watchdog_enable_caused_reboot()) {
        printf("This was a watchdog reboot - resetting\n");
        rom_reset_usb_boot(0, 0);
    }

    boot_info_t info;
    rom_get_boot_info(&info);
    printf("Boot partition: %d\n", info.partition);

    rom_connect_internal_flash();
    // rom_load_partition_table();

    uint32_t buf[3];
    int ret_num;
    ret_num = rom_get_partition_table_info(buf, count_of(buf), PT_INFO_SINGLE_PARTITION | (info.partition << 24) | PT_INFO_PARTITION_ID);
    if (ret_num != 3) {
        panic("Get partition ID failed %d", ret_num);
    }
    uint64_t my_id = (((uint64_t)buf[2]) << 32) | buf[1];
    printf("My ID is 0x%llx\n", my_id);

    uint32_t buffer[(16 * 4) + 1] = {}; // maximum of 16 partitions, each with maximum of 4 words returned, plus 1
    ret_num = rom_get_partition_table_info(buffer, count_of(buffer), PT_INFO_PARTITION_LOCATION_AND_FLAGS | PT_INFO_PARTITION_ID);

    int picked_p_start = -1;
    int picked_p_end = -1;
    if (ret_num > 0) {
        int i = 1;
        int p = 0;
        while (i < ret_num) {
            uint32_t location_and_permissions = buffer[i++];
            uint32_t flags_and_permissions = buffer[i++];
            bool has_id = (flags_and_permissions & PICOBIN_PARTITION_FLAGS_HAS_ID_BITS);
            if (has_id) {
                uint64_t id = 0;
                id |= buffer[i++];
                id |= ((uint64_t)(buffer[i++]) << 32ull);
                if (id == my_id && p != info.partition) {
                    picked_p_start = 0x1000 * ((location_and_permissions & PICOBIN_PARTITION_LOCATION_FIRST_SECTOR_BITS) >> PICOBIN_PARTITION_LOCATION_FIRST_SECTOR_LSB);
                    picked_p_end = 0x1000 * (1 + ((location_and_permissions & PICOBIN_PARTITION_LOCATION_LAST_SECTOR_BITS) >> PICOBIN_PARTITION_LOCATION_LAST_SECTOR_LSB));
                    break;
                }
            }

            p++;
        }
    }
    printf("Found matching partition %08x->%08x\n", picked_p_start, picked_p_end);

    #define FLASH_SECTOR_SHIFT 12u
    #define FLASH_SECTOR_SIZE (1ul << FLASH_SECTOR_SHIFT)
    #define FLASH_SECTOR_REMAINDER_MASK (FLASH_SECTOR_SIZE - 1u)

    int32_t roll = (int32_t)picked_p_start;
    if (roll) {
        printf("NEED TO ROLL %08x to %08x\n", (int)XIP_BASE + roll, XIP_BASE);
        if ((uint32_t)roll & FLASH_SECTOR_REMAINDER_MASK) {
            printf("CAN ONLY ROLL in sector multiples (4k)");
        }
        // window base is used during s_varm_crit_ram_trash_launch_image, so we need to roll it now
        roll >>= FLASH_SECTOR_SHIFT;
        int32_t size = (int32_t)((picked_p_end - picked_p_start) >> FLASH_SECTOR_SHIFT);
// 0x07ff0000 [26:16] : SIZE (0x400): Translation aperture size for this virtual address range, in units of 4 kiB (one...
// 0x00000fff [11:0]  : BASE (0): Physical address base for this virtual address range, in units of 4 kiB (one flash sector)
        for (uint i = 0; i < 4; i++) {
            static_assert(4 * 1024 * 1024 / FLASH_SECTOR_SIZE == 0x400, "");
            if (roll < 0) {
                roll += 0x400;
                qmi_hw->atrans[i] = 0;
            } else {
                int32_t this_size = MIN(size, 0x400);
                printf("ATRANS %d\n", i);
                qmi_hw->atrans[i] = (uint)((this_size << 16) | roll);
                size -= this_size;
                roll += this_size;
            }
        }
    }
    printf("ns_bin start: %08x\n", *(uint32_t*)XIP_BASE);

    int rc;

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
