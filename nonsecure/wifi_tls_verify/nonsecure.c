#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/bootrom.h"
#include "hardware/clocks.h"
#include "pico/cyw43_arch.h"
#include "hardware/dma.h"

// Using this url as we know the root cert won't change for a long time
#define TLS_CLIENT_SERVER "fw-download-alias1.raspberrypi.com"
#define TLS_CLIENT_HTTP_REQUEST  "GET /net_install/boot.sig HTTP/1.1\r\n" \
                                 "Host: " TLS_CLIENT_SERVER "\r\n" \
                                 "Connection: close\r\n" \
                                 "\r\n"
#define TLS_CLIENT_TIMEOUT_SECS  15

// This is the PUBLIC root certificate exported from a browser
// Note that the newlines are needed
#define TLS_ROOT_CERT_OK "-----BEGIN CERTIFICATE-----\n\
MIIC+jCCAn+gAwIBAgICEAAwCgYIKoZIzj0EAwIwgbcxCzAJBgNVBAYTAkdCMRAw\n\
DgYDVQQIDAdFbmdsYW5kMRIwEAYDVQQHDAlDYW1icmlkZ2UxHTAbBgNVBAoMFFJh\n\
c3BiZXJyeSBQSSBMaW1pdGVkMRwwGgYDVQQLDBNSYXNwYmVycnkgUEkgRUNDIENB\n\
MR0wGwYDVQQDDBRSYXNwYmVycnkgUEkgUm9vdCBDQTEmMCQGCSqGSIb3DQEJARYX\n\
c3VwcG9ydEByYXNwYmVycnlwaS5jb20wIBcNMjExMjA5MTEzMjU1WhgPMjA3MTEx\n\
MjcxMTMyNTVaMIGrMQswCQYDVQQGEwJHQjEQMA4GA1UECAwHRW5nbGFuZDEdMBsG\n\
A1UECgwUUmFzcGJlcnJ5IFBJIExpbWl0ZWQxHDAaBgNVBAsME1Jhc3BiZXJyeSBQ\n\
SSBFQ0MgQ0ExJTAjBgNVBAMMHFJhc3BiZXJyeSBQSSBJbnRlcm1lZGlhdGUgQ0Ex\n\
JjAkBgkqhkiG9w0BCQEWF3N1cHBvcnRAcmFzcGJlcnJ5cGkuY29tMHYwEAYHKoZI\n\
zj0CAQYFK4EEACIDYgAEcN9K6Cpv+od3w6yKOnec4EbyHCBzF+X2ldjorc0b2Pq0\n\
N+ZvyFHkhFZSgk2qvemsVEWIoPz+K4JSCpgPstz1fEV6WzgjYKfYI71ghELl5TeC\n\
byoPY+ee3VZwF1PTy0cco2YwZDAdBgNVHQ4EFgQUJ6YzIqFh4rhQEbmCnEbWmHEo\n\
XAUwHwYDVR0jBBgwFoAUIIAVCSiDPXut23NK39LGIyAA7NAwEgYDVR0TAQH/BAgw\n\
BgEB/wIBADAOBgNVHQ8BAf8EBAMCAYYwCgYIKoZIzj0EAwIDaQAwZgIxAJYM+wIM\n\
PC3wSPqJ1byJKA6D+ZyjKR1aORbiDQVEpDNWRKiQ5QapLg8wbcED0MrRKQIxAKUT\n\
v8TJkb/8jC/oBVTmczKlPMkciN+uiaZSXahgYKyYhvKTatCTZb+geSIhc0w/2w==\n\
-----END CERTIFICATE-----\n"

// This is a test certificate
#define TLS_ROOT_CERT_BAD "-----BEGIN CERTIFICATE-----\n\
MIIDezCCAwGgAwIBAgICEAEwCgYIKoZIzj0EAwIwgasxCzAJBgNVBAYTAkdCMRAw\n\
DgYDVQQIDAdFbmdsYW5kMR0wGwYDVQQKDBRSYXNwYmVycnkgUEkgTGltaXRlZDEc\n\
MBoGA1UECwwTUmFzcGJlcnJ5IFBJIEVDQyBDQTElMCMGA1UEAwwcUmFzcGJlcnJ5\n\
IFBJIEludGVybWVkaWF0ZSBDQTEmMCQGCSqGSIb3DQEJARYXc3VwcG9ydEByYXNw\n\
YmVycnlwaS5jb20wHhcNMjExMjA5MTMwMjIyWhcNNDYxMjAzMTMwMjIyWjA6MQsw\n\
CQYDVQQGEwJHQjErMCkGA1UEAwwiZnctZG93bmxvYWQtYWxpYXMxLnJhc3BiZXJy\n\
eXBpLmNvbTBZMBMGByqGSM49AgEGCCqGSM49AwEHA0IABJ6BQv8YtNiNv7ibLtt4\n\
lwpgEr2XD4sOl9wu/l8GnGD5p39YK8jZV0j6HaTNkqi86Nly1H7YklzbxhFy5orM\n\
356jggGDMIIBfzAJBgNVHRMEAjAAMBEGCWCGSAGG+EIBAQQEAwIGQDAzBglghkgB\n\
hvhCAQ0EJhYkT3BlblNTTCBHZW5lcmF0ZWQgU2VydmVyIENlcnRpZmljYXRlMB0G\n\
A1UdDgQWBBRlONP3G2wTERZA9D+VxJABfiaCVTCB5QYDVR0jBIHdMIHagBQnpjMi\n\
oWHiuFARuYKcRtaYcShcBaGBvaSBujCBtzELMAkGA1UEBhMCR0IxEDAOBgNVBAgM\n\
B0VuZ2xhbmQxEjAQBgNVBAcMCUNhbWJyaWRnZTEdMBsGA1UECgwUUmFzcGJlcnJ5\n\
IFBJIExpbWl0ZWQxHDAaBgNVBAsME1Jhc3BiZXJyeSBQSSBFQ0MgQ0ExHTAbBgNV\n\
BAMMFFJhc3BiZXJyeSBQSSBSb290IENBMSYwJAYJKoZIhvcNAQkBFhdzdXBwb3J0\n\
QHJhc3BiZXJyeXBpLmNvbYICEAAwDgYDVR0PAQH/BAQDAgWgMBMGA1UdJQQMMAoG\n\
CCsGAQUFBwMBMAoGCCqGSM49BAMCA2gAMGUCMEHerJRT0WmG5tz4oVLSIxLbCizd\n\
//SdJBCP+072zRUKs0mfl5EcO7dXWvBAb386PwIxAL7LrgpJroJYrYJtqeufJ3a9\n\
zVi56JFnA3cNTcDYfIzyzy5wUskPAykdrRrCS534ig==\n\
-----END CERTIFICATE-----\n"

extern bool run_tls_client_test(const uint8_t *cert, size_t cert_len, const char *server, const char *request, int timeout);

bool repeating_timer_callback(__unused struct repeating_timer *t) {
    printf("NS Repeat at %lld\n", time_us_64());
    return true;
}

int main() {
    printf("And lets begin\n");

    // Check no DMA channels are available
    int chan = dma_claim_unused_channel(false);
    printf("DMA channel: %d\n", chan);
    if (chan != -1) {
        printf("ERROR: DMA channel is already available\n");
        goto done;
    }
    // Request DMA channels from secure
    int num_dma_channels = dma_request_unused_channels_from_secure(8);
    printf("Got %d DMA channels\n", num_dma_channels);

    // Check no user IRQs are available
    int irq = user_irq_claim_unused(false);
    printf("User IRQ: %d\n", irq);
    if (irq != -1) {
        printf("ERROR: User IRQ is already available\n");
        goto done;
    }
    // Request user IRQs from secure
    int num_user_irqs = user_irq_request_unused_from_secure(1);
    printf("Got %d user IRQs\n", num_user_irqs);

    // Repeating timer
    struct repeating_timer timer;
    add_repeating_timer_ms(1000, repeating_timer_callback, NULL, &timer);

    int rc = cyw43_arch_init();
    printf("cyw43_arch_init rc=%d\n", rc);

    printf("My clock speed is %dHz\n", clock_get_hz(clk_sys));

    bool led_on = true;
    for (int i=0; i < 5; i++) {
        printf("Hello, world, from non-secure!\n");
        cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
        led_on = !led_on;
        sleep_ms(1000);
    }

    cyw43_arch_enable_sta_mode();

    if (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 30000)) {
        printf("failed to connect\n");
        goto done;
    }

    // This should work
    printf("Running TLS client test with good certificate\n");
    const uint8_t cert_ok[] = TLS_ROOT_CERT_OK;
    bool pass1 = run_tls_client_test(cert_ok, sizeof(cert_ok), TLS_CLIENT_SERVER, TLS_CLIENT_HTTP_REQUEST, TLS_CLIENT_TIMEOUT_SECS);
    if (pass1) {
        printf("Test passed\n");
    } else {
        printf("Test failed\n");
    }

    // Repeat the test with the wrong certificate. It should fail
    printf("Running TLS client test with bad certificate\n");
    const uint8_t cert_bad[] = TLS_ROOT_CERT_BAD;
    bool pass2 = !run_tls_client_test(cert_bad, sizeof(cert_bad), TLS_CLIENT_SERVER, TLS_CLIENT_HTTP_REQUEST, TLS_CLIENT_TIMEOUT_SECS);
    if (pass2) {
        printf("Test passed\n");
    } else {
        printf("Test failed\n");
    }

done:
    printf("Triggering secure fault\n");

    volatile uint32_t thing = *(uint32_t*)SRAM4_BASE;

    printf("SRAM4 is %08x\n", thing);

    rom_reset_usb_boot(0, 0);

    return 0;
}
