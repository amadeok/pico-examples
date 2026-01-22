#pragma once

#define UART_ID         uart0
#define BAUD_RATE       115200
#define UART_TX_PIN     0
#define UART_RX_PIN     1
#define LED_PIN         PICO_DEFAULT_LED_PIN  // = 25 on Pico

#define MAX_PAYLOAD     20
#define CRC8_POLY       0x07            // Very common polynomial (CRC-8)

#define BAUD_RATE 115200
#define DATA_BITS 8
#define STOP_BITS 1
#define PARITY    UART_PARITY_NONE
#define QUEUE_LENGTH 64

#define HAS_KEYBOARD  1
#define HAS_MOUSE     1

int tx_error(uint8_t op);
