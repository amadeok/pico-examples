#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "pico/util/queue.h"
#include <string.h>
#include "hello_uart.h"
#include "bsp/board_api.h"
#include "tusb.h"
#include "usb_descriptors.h"
#include "pico/sync.h" // ← contains mutex.h



mutex_t my_mutex;

queue_t mouse_queue;
queue_t keyboard_queue;
uint32_t counter = 0;

typedef enum
{
    STATE_WAIT_LEN,
    STATE_PAYLOAD,
    STATE_WAIT_CRC
} rx_state_t;

typedef enum
{
    INFO_REPORT_REQUEST = 0xFE,
    INFO_REPORT = 0xFF,
    ERROR_REPORT = 0xFD
} operation_t;

#define MAX_PACKET     (MAX_PAYLOAD + 2)
static volatile rx_state_t rx_state = STATE_WAIT_LEN;
static uint8_t rx_len_expected = 0;
static uint8_t rx_buf[MAX_PACKET]; // length + payload + crc
static uint8_t rx_pos = 0;
static uint8_t tx_buf[MAX_PACKET]; // length + payload + crc


typedef struct {
    uint8_t length;                         // actual used bytes in .data
    uint8_t data[MAX_PACKET];              // or just MAX_PAYLOAD if you store length separately
} message_t;
#define QUEUE_DEPTH     100 
                 // usually 8–32 is plenty — each slot = ~66 bytes
static queue_t msg_queue;

uint32_t invalid_length_errors = 0;
uint32_t crc_failed_errors = 0;

uint8_t queue_send_report = 0;

static uint8_t crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    while (len--)
    {
        crc ^= *data++;
        for (uint8_t i = 0; i < 8; i++)
        {
            crc = (crc & 0x80) ? (crc << 1) ^ CRC8_POLY : crc << 1;
        }
    }
    return crc;
}

void blink(int times, int delay)
{
    for (int i = 0; i < times; i++)
    {
        gpio_put(LED_PIN, 1);
        sleep_ms(delay);
        gpio_put(LED_PIN, 0);
        sleep_ms(delay);
    }
}

void build_error_payload(uint8_t *buf, uint8_t op)
{

    buf[0] = op;

    buf[1] = 0;
#if HAS_MOUSE
    buf[1] |= REPORT_ID_MOUSE;
#endif
#if HAS_KEYBOARD
    buf[1] |= REPORT_ID_KEYBOARD;
#endif

#if HAS_MOUSE
    buf[2] = queue_get_level(&mouse_queue);
#endif

#if HAS_KEYBOARD
    buf[3] = queue_get_level(&keyboard_queue);
#endif

    memcpy(buf + 4, &invalid_length_errors, 4);
    memcpy(buf + 8, &crc_failed_errors, 4);
}

size_t send_msg(const uint8_t *payload, size_t payload_len)
{

    uint8_t packet_len = payload_len +2;

    message_t m;

    uint8_t * pkt = m.data; //[32];
    pkt[0] = payload_len;
    for (uint8_t i = 0; i < payload_len; i++)
    {
        pkt[1 + i] = payload[i];
    }
    pkt[1 + payload_len] = crc8(pkt, payload_len + 1);

    // for (int i = 0; i < len + 2; i++)
        // queue_add_blocking(&msg_queue, pkt + i);
    // uart_write_blocking(UART_ID, pkt, packet_len);

    m.length = packet_len;
    // memcpy(m.data, pkt, packet_len);
    bool added = queue_try_add(&msg_queue, &m);

    return packet_len;
}

// Called from IRQ — keep it fast!
void on_uart_rx()
{
    while (uart_is_readable(UART_ID))
    {
        uint8_t byte = uart_getc(UART_ID);

        switch (rx_state)
        {
        case STATE_WAIT_LEN:
            if (byte == 0 || byte > MAX_PAYLOAD)
            {
                // invalid length → resync (discard)
                invalid_length_errors++;
                  tx_error(ERROR_REPORT);
                // queue_send_report = ERROR_REPORT;
                continue;
            }
            rx_len_expected = byte;
            rx_buf[0] = byte;
            rx_pos = 1;
            rx_state = STATE_PAYLOAD;
            break;

        case STATE_PAYLOAD:
            rx_buf[rx_pos++] = byte;
            if (rx_pos == rx_len_expected + 1)
            {
                rx_state = STATE_WAIT_CRC;
            }
            break;

        case STATE_WAIT_CRC:
            rx_buf[rx_pos] = byte; // store received CRC

            uint8_t calc_crc = crc8(rx_buf, rx_len_expected + 1);
            if (calc_crc == byte)
            {
                // Payload starts at &rx_buf[1], length = rx_len_expected

                // Example: echo back the same message
                uint8_t len = rx_len_expected;
                uint8_t *payload = &rx_buf[1];


                send_msg(payload, rx_len_expected);

                uint8_t op = payload[0];
                switch (op)
                {
                case REPORT_ID_MOUSE:
                    hid_mouse_report_t m_elem;
                    memcpy(&m_elem, payload + 1, sizeof(hid_mouse_report_t));
                    queue_try_add(&mouse_queue, &m_elem);
                    break;
                case REPORT_ID_KEYBOARD:
                    hid_keyboard_report_t k_elem;
                    memcpy(&k_elem, payload + 1, sizeof(hid_keyboard_report_t));
                    queue_try_add(&keyboard_queue, &k_elem);
                    break;
                case INFO_REPORT_REQUEST:
                {
                    tx_error(INFO_REPORT);
                }
                // queue_send_report = INFO_REPORT;
                break;
                }
            }
            else
            {
                crc_failed_errors++;
                  tx_error(ERROR_REPORT);
                // queue_send_report = ERROR_REPORT;
            }

            // Reset for next packet
            rx_state = STATE_WAIT_LEN;
            rx_pos = 0;
            break;
        }
    }
}

int tx_error(uint8_t op)
{
    build_error_payload(tx_buf, op);
    uint8_t len = 12;
    uint8_t *payload = &tx_buf[0];
    send_msg(payload, len);
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const *buffer, uint16_t bufsize)
{
    (void)instance;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t *buffer, uint16_t reqlen)
{
    // TODO not Implemented
    (void)instance;
    (void)report_id;
    (void)report_type;
    (void)buffer;
    (void)reqlen;

    return 0;
}

static void send_hid_report(uint8_t report_id)
{
    // skip if hid is not ready yet

    if (!tud_hid_ready())
        return;

    switch (report_id)
    {
    case REPORT_ID_KEYBOARD:
    {
        hid_keyboard_report_t k_elem;
        if (queue_try_remove(&keyboard_queue, &k_elem))
        {
            tud_hid_keyboard_report(REPORT_ID_KEYBOARD, k_elem.modifier, k_elem.keycode);
        }
        // Optional: send empty report only when queue becomes empty
        // else tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, NULL);
    }
    break;

    case REPORT_ID_MOUSE:
    {
        hid_mouse_report_t m_elem;
        if (queue_try_remove(&mouse_queue, &m_elem))
        {
            tud_hid_mouse_report(REPORT_ID_MOUSE,
                                 m_elem.buttons,
                                 m_elem.x, m_elem.y,
                                 m_elem.wheel, m_elem.pan);
        }
    }
    break;

    default:
        break;
    }
}

void send_report(void)
{

#if HAS_MOUSE
    if (!queue_is_empty(&mouse_queue))
    {
        send_hid_report(REPORT_ID_MOUSE);
        return;
    }
#endif
#if HAS_KEYBOARD
    if (!queue_is_empty(&keyboard_queue))
    {
        send_hid_report(REPORT_ID_KEYBOARD);
        return;
    }
#endif
}

void tud_hid_report_complete_cb(uint8_t instance, uint8_t const *report, uint16_t len)
{
    (void)instance;
    (void)len;

    send_report();
}

void hid_task(void)
{

    const uint32_t interval_ms = 1;
    static uint32_t start_ms = 0;

    if (board_millis() - start_ms < interval_ms)
        return; // not enough time
    start_ms += interval_ms;

    if (tud_suspended())
    {
        // Wake up host if we are in suspend mode
        // and REMOTE_WAKEUP feature is enabled by host
        tud_remote_wakeup();
    }
    else
    {
        // Send the 1st of report chain, the rest will be sent by tud_hid_report_complete_cb()
        send_report();
    }
}

int main()
{
    board_init();
    mutex_init(&my_mutex);

    tud_init(BOARD_TUD_RHPORT);
    if (board_init_after_tusb)
    {
        board_init_after_tusb();
    }

#if HAS_MOUSE
    queue_init(&mouse_queue, sizeof(hid_mouse_report_t), QUEUE_LENGTH);
#endif
#if HAS_KEYBOARD
    queue_init(&keyboard_queue, sizeof(hid_keyboard_report_t), QUEUE_LENGTH);
#endif

    queue_init(&msg_queue, sizeof(message_t), 100);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    gpio_put(LED_PIN, 0);
    // stdio_init_all();   // optional — for debug printf over USB

    // Init UART
    uart_init(UART_ID, BAUD_RATE);

    gpio_set_function(UART_TX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_TX_PIN));
    gpio_set_function(UART_RX_PIN, UART_FUNCSEL_NUM(UART_ID, UART_RX_PIN));

    int __unused actual = uart_set_baudrate(UART_ID, BAUD_RATE);
    uart_set_hw_flow(UART_ID, false, false);
    uart_set_format(UART_ID, DATA_BITS, STOP_BITS, PARITY);
    uart_set_fifo_enabled(UART_ID, false);
    int UART_IRQ = UART_ID == uart0 ? UART0_IRQ : UART1_IRQ;

    irq_set_exclusive_handler(UART_IRQ, on_uart_rx);
    irq_set_enabled(UART_IRQ, true);

    uart_set_irq_enables(UART_ID, true, false);
    // uart_puts(UART_ID, "\nHello, uart interrupts\n");


    // message_t msg;
    message_t msg;
    while (true)
    {
        tud_task();
        counter++;
        hid_task();

        if (queue_try_remove(&msg_queue, &msg))
        {

            uart_write_blocking(UART_ID, msg.data, msg.length);
        }


        sleep_ms(1);
    }

    return 0;
}
