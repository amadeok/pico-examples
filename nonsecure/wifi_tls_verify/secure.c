#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "boot/picobin.h"
#include "pico/rand.h"
#include "hardware/watchdog.h"

#include "hardware/structs/dma.h"
#include "hardware/structs/sau.h"
#include "hardware/structs/qmi.h"
#include "hardware/exception.h"
#include "hardware/structs/scb.h"


int rom_c_callback(uint32_t a, uint32_t b, uint32_t c, uint32_t d, uint32_t fn) {
    // printf("Secure call happened\n");
    // printf("%08x\n", a);
    // printf("%08x\n", b);
    // printf("%08x\n", c);
    // printf("%08x\n", d);
    // printf("%08x\n", fn);
    switch (fn) {
        case 0: {
            // printf("Printing string\n");
            printf("%s", (char*)a);
            return 0;
        }
        case 1: {
            printf("get_rand_%d(%p)\n", a, b);
            switch (a) {
                case 32:
                    return get_rand_32();
                case 64:
                    return get_rand_64();
                case 128:
                    get_rand_128((rng_128_t*)b);
                    return 0;
                default:
                    printf("%d is an unknown rand function\n", a);
                    return BOOTROM_ERROR_NOT_PERMITTED;
            }
        }
        default: {
            printf("%d is not a supported rom function - sorry\n", fn);
            return BOOTROM_ERROR_NOT_PERMITTED;
        }

    }
}


int __attribute__((naked)) rom_callback() {
    pico_default_asm_volatile(
            "push {r0, lr}\n"
            "str r4, [sp]\n"
            "bl rom_c_callback\n"
            "pop {r1, pc}\n"
    );
}


bool repeating_timer_callback(__unused struct repeating_timer *t) {
    printf("Repeat at %lld\n", time_us_64());
    watchdog_update();

#if 0
    uint32_t sp;
    pico_default_asm_volatile(
        "mrs %0, msp_ns"
        : "=r" (sp)
    );

    printf("sp %08x\n", sp);
    printf("r0 %08x\n", *((uint32_t*)sp + 0));
    printf("r1 %08x\n", *((uint32_t*)sp + 1));
    printf("r2 %08x\n", *((uint32_t*)sp + 2));
    printf("r3 %08x\n", *((uint32_t*)sp + 3));
    printf("r12 %08x\n", *((uint32_t*)sp + 4));
    printf("lr %08x\n", *((uint32_t*)sp + 5));
    printf("pc %08x\n", *((uint32_t*)sp + 6));
    printf("xPSR %08x\n", *((uint32_t*)sp + 7));
#endif

    return true;
}


void hard_fault_handler(void) {
    printf("Hard fault occurred at %lld, resetting\n", time_us_64());

    // # First eight values on stack will always be:
    // # r0, r1, r2, r3, r12, LR, pc, xPSR
    // (gdb) p/a *(uint32_t[8] *)$psp

    uint32_t sp;
    pico_default_asm_volatile(
        "mrs %0, msp_ns"
        : "=r" (sp)
    );

    printf("sp %08x\n", sp);
    printf("r0 %08x\n", *((uint32_t*)sp + 0));
    printf("r1 %08x\n", *((uint32_t*)sp + 1));
    printf("r2 %08x\n", *((uint32_t*)sp + 2));
    printf("r3 %08x\n", *((uint32_t*)sp + 3));
    printf("r12 %08x\n", *((uint32_t*)sp + 4));
    printf("lr %08x\n", *((uint32_t*)sp + 5));
    printf("pc %08x\n", *((uint32_t*)sp + 6));
    printf("xPSR %08x\n", *((uint32_t*)sp + 7));

    printf("HFSR %08x\n", scb_hw->hfsr);
    if (scb_hw->hfsr & M33_HFSR_DEBUGEVT_BITS) printf("HardFault Debug Event\n");
    if (scb_hw->hfsr & M33_HFSR_FORCED_BITS) printf("HardFault Forced\n");

    printf("SFSR %08x\n", m33_hw->sfsr);
    if (m33_hw->sfsr & M33_SFSR_AUVIOL_BITS) printf("SecureFault Non-secure accessed Secure\n");
    if (m33_hw->sfsr & M33_SFSR_INVEP_BITS) printf("SecureFault Non-secure branched Secure\n");
    if (m33_hw->sfsr & M33_SFSR_SFARVALID_BITS) printf("SFAR %08x\n", m33_hw->sfar);

    printf("CFSR %08x\n", scb_hw->cfsr);
    if (scb_hw->cfsr & M33_CFSR_UFSR_NOCP_BITS) printf("UsageFault No Coprocessor\n");
    if (scb_hw->cfsr & (M33_CFSR_BFSR_IMPRECISERR_BITS | M33_CFSR_BFSR_PRECISERR_BITS)) printf("BusFault Data Error\n");
    printf("MMFAR %08x\n", scb_hw->mmfar);
    if (m33_hw->cfsr & M33_CFSR_BFSR_BFARVALID_BITS) printf("BFAR %08x\n", scb_hw->bfar);

    printf("DHCSR %08x\n", m33_hw->dhcsr);

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

    // Enable NS timer access
    accessctrl_hw->timer[1] |= 0xacce0000 | ACCESSCTRL_TIMER1_NSP_BITS | ACCESSCTRL_TIMER1_NSU_BITS;

    // Enable NS timer IRQs
    irq_assign_to_ns(TIMER1_IRQ_0, true);
    irq_assign_to_ns(TIMER1_IRQ_1, true);
    irq_assign_to_ns(TIMER1_IRQ_2, true);
    irq_assign_to_ns(TIMER1_IRQ_3, true);

    // Enable NS GPIO access
    accessctrl_hw->gpio_nsmask[0] = 0xfffffffc;
    accessctrl_hw->gpio_nsmask[1] = 0xffffffff;
    accessctrl_hw->pads_bank0 |= 0xacce0000 | ACCESSCTRL_PADS_BANK0_NSP_BITS | ACCESSCTRL_PADS_BANK0_NSU_BITS;
    accessctrl_hw->io_bank[0] |= 0xacce0000 | ACCESSCTRL_PADS_BANK0_NSP_BITS | ACCESSCTRL_PADS_BANK0_NSU_BITS;

    // Enable NS GPIO IRQ
    irq_assign_to_ns(IO_IRQ_BANK0_NS, true);

    // Enable NS DMA access
    accessctrl_hw->dma |= 0xacce0000 | ACCESSCTRL_DMA_NSP_BITS | ACCESSCTRL_DMA_NSU_BITS;

    // Enable SAU
    __dmb();

    // XIP is NS Code
    sau_hw->rnr = 0;
    sau_hw->rbar = (XIP_BASE & M33_SAU_RBAR_BADDR_BITS);
    sau_hw->rlar = ((XIP_END - 1) & M33_SAU_RLAR_LADDR_BITS) | M33_SAU_RLAR_ENABLE_BITS;
    printf("RNR 0, RBAR %08x, RLAR %08x\n", sau_hw->rbar, sau_hw->rlar);

    // SRAM0-3 (up to 2003000) is NS data
    sau_hw->rnr = 1;
    sau_hw->rbar = (SRAM_BASE & M33_SAU_RBAR_BADDR_BITS);
    sau_hw->rlar = ((0x20030000 - 1) & M33_SAU_RLAR_LADDR_BITS) | M33_SAU_RLAR_ENABLE_BITS;
    printf("RNR 1, RBAR %08x, RLAR %08x\n", sau_hw->rbar, sau_hw->rlar);

    // SRAM8-9 is NS scratch
    sau_hw->rnr = 2;
    sau_hw->rbar = (SRAM8_BASE & M33_SAU_RBAR_BADDR_BITS);
    sau_hw->rlar = ((SRAM_END - 1) & M33_SAU_RLAR_LADDR_BITS) | M33_SAU_RLAR_ENABLE_BITS;
    printf("RNR 2, RBAR %08x, RLAR %08x\n", sau_hw->rbar, sau_hw->rlar);

    // Check bootrom configured region
    sau_hw->rnr = 7;
    sau_hw->rbar = 0x46a0;
    sau_hw->rlar = 0x7fe1;
    printf("RNR 7, RBAR %08x, RLAR %08x\n", sau_hw->rbar, sau_hw->rlar);

    sau_hw->ctrl = M33_SAU_CTRL_ENABLE_BITS;

    __dsb();
    __isb();

    printf("SAU Configured\n");

    exception_handler_t old_handler = exception_set_exclusive_handler(HARDFAULT_EXCEPTION, hard_fault_handler);
    printf("HardFault Handler set to %08x from %p\n", hard_fault_handler, old_handler);

    struct repeating_timer timer;
    watchdog_enable(2000, true);
    add_repeating_timer_ms(1000, repeating_timer_callback, NULL, &timer);

    scb_ns_hw->vtor = XIP_BASE + 0x200;

    pico_default_asm(
        "msr msplim, %0\n"
        "msr msp_ns, %1\n"
        "msr msplim_ns, %2\n"
        "movs r1, %3\n"
        "movs r0, %4\n"
        // "movs r2, #0\n"
        // "ldmia r3, {r1-r12}\n"
        "blxns r1"
        :
        :   "r" (SRAM8_BASE - 0x1000),
            "r" (SRAM_END),
            "r" (SRAM8_BASE),
            "r" ((XIP_BASE & ~1) + 4),  // Jump into _enter_vtable_in_r0, not _entry_point
            "r" (scb_ns_hw->vtor)      // And put vtable in r0
    );

    printf("Shouldn't return from jump\n");
}
