/*
* Project 'raspi-pico-aprs-tnc'
* Copyright (C) 2021-2023 Thomas Glau, DL3TG
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.

* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.

* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

/* This program demonstrates the usage of the 'libaprs_pico.a' library by
 * showing how to send a static APRS beacon.
 *
 * Optionally, PTT control can be enabled (see the #define section down below).
 */

// Define whether/how the RPi Pico should control a transmitter's PTT input
#define PTT_ENABLE (true)
#define PTT_GPXX_PIN (19)
#define PD_GPXX_PIN (22)
#define PTT_DELAY_BEFORE_TX_IN_MSEC (1000)
#define PTT_TX_PERIOD_IN_MIN (1)

#include "aprs_pico.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <stdbool.h>
#include <stdio.h>

// SA818
#define UART_ID uart1
#define BAUD_RATE 9600
#define UART_TX_PIN 4
#define UART_RX_PIN 5

// GPS
#define UART_ID uart0
#define BAUD_RATE 9600
#define UART_TX_PIN 12
#define UART_RX_PIN 13

bool configSa818(const char *freq) {
  printf("\n--- SA818 Configuration Mode ---\n");

  // 1. Handshake / Connection Check
  // The SA818 responds with "+DMOSETGROUP:0" if awake
  uart_puts(UART_ID, "AT+DMOCONNECT\r\n");
  sleep_ms(200);

  if (uart_is_readable(UART_ID)) {
    printf("Response: ");
    while (uart_is_readable(UART_ID)) {
      char c = uart_getc(UART_ID);
      putchar(c); // This prints the radio's response (e.g., +DMOCONNECT:0) to
                  // your terminal
    }
    printf("\n");
  } else {
    printf("No response from SA818. Check PD pin and wiring!\n");
  }

  // 2. Setup Group (Frequency, Bandwidth, etc.)
  // Syntax: AT+DMOSETGROUP=GBW,TFV,RFV,TX_CXCSS,SQ,RX_CXCSS
  // GBW: 0 = 12.5K (Narrow), 1 = 25K (Wide). APRS is usually Narrow (0).
  // TX_CXCSS / RX_CXCSS: 0000 = No CTCSS/DCS
  // SQ: 0-8 (4 is standard)

  char command[64];
  // We format the string: 0 (BW), freq (TX), freq (RX), 0000 (TX_CTCSS), 4
  // (SQ), 0000 (RX_CTCSS)
  snprintf(command, sizeof(command), "AT+DMOSETGROUP=0,%s,%s,0000,4,0000\r\n",
           freq, freq);

  printf("Sending: %s", command);
  uart_puts(UART_ID, command);

  // 3. Wait for response
  sleep_ms(500);
  bool success = false;
  printf("Response: ");

  while (uart_is_readable(UART_ID)) {
    char c = uart_getc(UART_ID);
    putchar(c);
    // SA818 returns "+DMOSETGROUP:0" on success
    if (c == '0')
      success = true;
  }
  printf("\n");

  // 4. Set Volume (Optional but recommended for APRS)
  // AT+DMOSETVOLUME=x (x=1 to 8)
  uart_puts(UART_ID, "AT+DMOSETVOLUME=5\r\n");
    while (uart_is_readable(UART_ID)) {
    putchar(uart_getc(UART_ID));
  }

  uart_puts(UART_ID, "AT+SETFILTER=1,1,1\r\n");
sleep_ms(200);
while (uart_is_readable(UART_ID)) {
    putchar(uart_getc(UART_ID)); // Expect +SETFILTER:0
}

  return success;
}

int main() {
  stdio_init_all();

  for (int i = 0; i < 10; i++) {
    if (stdio_usb_connected())
      break;
    sleep_ms(100);
  }

  gpio_init(PTT_GPXX_PIN);
  gpio_set_dir(PTT_GPXX_PIN, GPIO_OUT);
  gpio_init(PD_GPXX_PIN);
  gpio_set_dir(PD_GPXX_PIN, GPIO_OUT);
  gpio_put(PD_GPXX_PIN, true); // Force Receive mode so UART works

  // 2. Initialize Hardware UART for the Radio
  uart_init(UART_ID, BAUD_RATE);
  gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);

  // 3. Run your config
  if (configSa818("144.8000")) {
    printf("Radio configured successfully!\n");
  } else {
    printf("Radio config failed.\n");
  }

  uart_puts(UART_ID, "AT+DMOREADGROUP\r\n");
  sleep_ms(200);
  while (uart_is_readable(UART_ID)) {
    putchar(uart_getc(UART_ID));
  }


  // ----------------------------------
  gpio_init(18);
gpio_set_dir(18, GPIO_OUT);
gpio_put(PTT_GPXX_PIN, false); // KEY RADIO

for(int i = 0; i < 500; i++) {
    gpio_put(18, 1);
    sleep_us(400); // Create a ~1200Hz square wave
    gpio_put(18, 0);
    sleep_us(400);
}

gpio_put(PTT_GPXX_PIN, true); // UNKEY
//--------------------------------


#if PTT_ENABLE == true

  gpio_init(PTT_GPXX_PIN);
  gpio_set_dir(PTT_GPXX_PIN, GPIO_OUT);
  sleep_ms(200);

#endif // PTT_ENABLE

  audio_buffer_pool_t *audio_buffer_pool = aprs_pico_init();

  // Loop forever
  while (true) {

#if PTT_ENABLE == true

    gpio_put(PTT_GPXX_PIN, false);
    sleep_ms(PTT_DELAY_BEFORE_TX_IN_MSEC);

#endif // PTT_ENABLE

    // Send an APRS test message
    aprs_pico_sendAPRS(
        audio_buffer_pool,
        "OE6UKN-12", // Source call sign
        "APPIPI",   // Destination call sign
        "WIDE1-1",  // APRS path #1
        "WIDE2-2",  // APRS path #2
        "APRS by RPi-Pico - "
        "https://github.com/eleccoder/raspi-pico-aprs-tnc", // Text
                                                            // message
        47.3805128, // Latitude  (in deg)
        15.0947756, // Longitude (in deg)
        0,          // Altitude  (in m)
        '/',        // APRS symbol table: Primary
        '>',        // APRS symbol code:  Car
        128u);      // Volume    (0 ... 256)

#if PTT_ENABLE == true

    gpio_put(PTT_GPXX_PIN, true);
    sleep_ms(PTT_TX_PERIOD_IN_MIN * 60 * 1000);

#endif // PTT_ENABLE

  }

  return 0;
}