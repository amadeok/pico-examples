#include <stdio.h>
#include "pico/bootrom.h"
#include "hardware/clocks.h"
#include "pico/stdlib.h"

bool repeating_timer_callback(__unused struct repeating_timer *t) {
    printf("NS Repeat at %lld\n", time_us_64());
    return true;
}

int main() {
    // Check no user IRQs are available
    int irq = user_irq_claim_unused(false);
    if (irq != -1) {
        goto done;
    }
    // Request user IRQs from secure, which stdio_usb will use
    user_irq_request_unused_from_secure(1);

    stdio_usb_init();

    // Repeating timer
    struct repeating_timer timer;
    add_repeating_timer_ms(1000, repeating_timer_callback, NULL, &timer);

    printf("My clock speed is %dHz\n", clock_get_hz(clk_sys));

    for (int i=0; i < 10; i++) {
        printf("Hello, world, from non-secure!\n");
        sleep_ms(1000);
    }

done:
    printf("Triggering secure fault by reading secure memory\n");
    // Secure memory is the rest of SRAM after the heap limit
    extern uint32_t __HeapLimit;
    volatile uint32_t thing = *(uint32_t*)__HeapLimit;
    printf("Some secure memory is %08x\n", thing);

    return 0;
}
