#include <stdio.h>
#include "pico/bootrom.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"

#include "secure_call_user_callbacks.h"

bool repeating_timer_callback(__unused struct repeating_timer *t) {
    printf("NS Repeat at %lld\n", time_us_64());
    return true;
}

int main() {
    // Request user IRQ from secure, which stdio_usb will use
    user_irq_request_unused_from_secure(1);

    stdio_usb_init();

    // Repeating timer
    struct repeating_timer timer;
    add_repeating_timer_ms(1000, repeating_timer_callback, NULL, &timer);

    for (int i=0; i < 10; i++) {
        printf("Hello, world, from non-secure!\n");
        rom_secure_call(1, 2, 3, i, SECURE_CALL_PRINT_VALUES);
        sleep_ms(1000);
    }

    // Demonstrate triggering a secure fault, by reading secure memory at start of SRAM
    printf("Triggering secure fault by reading secure memory\n");
    volatile uint32_t thing = *(uint32_t*)SRAM_BASE;
    printf("Some secure memory is %08x\n", thing);

    return 0;
}
