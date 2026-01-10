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

#include "aprs_pico.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

// Define whether/how the RPi Pico should control a transmitter's PTT input
#define PTT_ENABLE (true)
#define PTT_GPXX_PIN (19)
#define PD_GPXX_PIN (22)
#define PTT_DELAY_BEFORE_TX_IN_MSEC (1000)
#define PTT_TX_PERIOD_IN_MIN (10)

// SA818
#define RADIO_UART uart1
#define RADIO_BAUD 9600
#define RADIO_TX_PIN 4
#define RADIO_RX_PIN 5

// GPS
#define GPS_UART uart0
#define GPS_BAUD 9600
#define GPS_TX_PIN 12
#define GPS_RX_PIN 13

typedef struct {
  double lat;
  double lon;
  float speed_knots;
  bool valid;
} gps_data_t;

bool ConfigureRadio(const char *freq) {
  printf("\n--- SA818 Configuration Mode ---\n");

  gpio_init(PD_GPXX_PIN);
  gpio_set_dir(PD_GPXX_PIN, GPIO_OUT);
  gpio_put(PD_GPXX_PIN, true);

  // 1. Handshake / Connection Check
  // The SA818 responds with "+DMOSETGROUP:0" if awake
  uart_puts(RADIO_UART, "AT+DMOCONNECT\r\n");
  sleep_ms(200);

  if (uart_is_readable(RADIO_UART)) {
    printf("Response: ");
    while (uart_is_readable(RADIO_UART)) {
      char c = uart_getc(RADIO_UART);
      putchar(c);
    }
    printf("\n");
  } else {
    printf("No response from SA818. Check PD pin and wiring!\n");
  }

  // 2. Setup Group (Frequency, Bandwidth, etc.)
  // Syntax: AT+DMOSETGROUP=GBW,TFV,RFV,TX_CXCSS,SQ,RX_CXCSS

  char command[64];
  snprintf(command, sizeof(command), "AT+DMOSETGROUP=0,%s,%s,0000,4,0000\r\n",
           freq, freq);

  printf("Sending: %s", command);
  uart_puts(RADIO_UART, command);

  // 3. Wait for response
  bool success = false;
  printf("Response: ");

  while (uart_is_readable(RADIO_UART)) {
    char c = uart_getc(RADIO_UART);
    putchar(c);
    // SA818 returns "+DMOSETGROUP:0" on success
    if (c == '0')
      success = true;
  }
  printf("\n");

  // 4. Set Volume (Optional but recommended for APRS)
  // AT+DMOSETVOLUME=x (x=1 to 8)
  uart_puts(RADIO_UART, "AT+DMOSETVOLUME=5\r\n");
  while (uart_is_readable(RADIO_UART)) {
    putchar(uart_getc(RADIO_UART));
  }

  // uart_puts(RADIO_UART, "AT+SETFILTER=1,1,1\r\n");
  // sleep_ms(200);
  // while (uart_is_readable(RADIO_UART)) {
  //   putchar(uart_getc(RADIO_UART)); // Expect +SETFILTER:0
  // }

  return success;
}

// Helper to convert NMEA DDMM.MMMM to Decimal Degrees
double nmea_to_decimal(float nmea_coord, char direction) {
  int degrees = (int)(nmea_coord / 100);
  double minutes = nmea_coord - (degrees * 100);
  double decimal = degrees + (minutes / 60.0);
  if (direction == 'S' || direction == 'W')
    decimal *= -1;
  return decimal;
}

gps_data_t parse_gprmc(char *line) {
  gps_data_t data = {0, 0, 0, false};

  // 1. Look for RMC anywhere in the string
  char *start = strstr(line, "RMC");
  if (!start)
    return data;

  // 2. Create a working copy because strsep modifies the string
  char work_line[128];
  strncpy(work_line, start, sizeof(work_line));

  char *p = work_line;
  char *token;
  int field = 0;

  while ((token = strsep(&p, ",")) != NULL) {
    // field 0 is "RMC" (the remainder of GNRMC)
    // field 1 is Time
    if (field == 2) { // Status: A=Active, V=Void
      if (token[0] != 'A')
        return data;
    }
    if (field == 3 && strlen(token) > 0)
      data.lat = atof(token);
    if (field == 4 && strlen(token) > 0)
      data.lat = nmea_to_decimal(data.lat, token[0]);
    if (field == 5 && strlen(token) > 0)
      data.lon = atof(token);
    if (field == 6 && strlen(token) > 0)
      data.lon = nmea_to_decimal(data.lon, token[0]);
    if (field == 7 && strlen(token) > 0)
      data.speed_knots = atof(token);

    field++;
  }

  // Only mark valid if we actually got coordinates
  if (data.lat != 0 && data.lon != 0) {
    data.valid = true;
  }

  return data;
}

int main() {
  stdio_init_all();

  for (int i = 0; i < 10; i++) {
    if (stdio_usb_connected())
      break;
    sleep_ms(100);
  }

  // Force Receive mode so UART works
  gpio_init(PTT_GPXX_PIN);
  gpio_set_dir(PTT_GPXX_PIN, GPIO_OUT);
  gpio_put(PTT_GPXX_PIN, true);
  sleep_ms(200);

  // 2. Initialize Hardware UART for the Radio
  uart_init(RADIO_UART, RADIO_BAUD);
  gpio_set_function(RADIO_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(RADIO_RX_PIN, GPIO_FUNC_UART);

  // 2. Initialize Hardware UART for the GPS
  uart_init(GPS_UART, GPS_BAUD);
  gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
  gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);

  // 3. Run your config
  if (ConfigureRadio("144.8000")) {
    printf("Radio configured successfully!\n");
  } else {
    printf("Radio config failed.\n");
  }

  uart_puts(RADIO_UART, "AT+DMOREADGROUP\r\n");
  sleep_ms(200);
  while (uart_is_readable(RADIO_UART)) {
    putchar(uart_getc(RADIO_UART));
  }

  audio_buffer_pool_t *audio_buffer_pool = aprs_pico_init();
  uint32_t last_tx_time = 0;
  const uint32_t tx_interval_ms =
      PTT_TX_PERIOD_IN_MIN * 60 * 1000; // 10 minutes

  char gps_buffer[128];
  int gps_idx = 0;

  // Loop forever
  while (true) {
    gps_data_t current_gps;

    // Inside your while(true) loop:
    while (uart_is_readable(GPS_UART)) {
      char c = uart_getc(GPS_UART);
      // putchar(c); // Print raw NMEA sentences to your USB console
      if (c == '\n' || gps_idx > 126) {
        gps_buffer[gps_idx] = '\0';
        gps_data_t new_data = parse_gprmc(gps_buffer);
        if (new_data.valid) {
          current_gps = new_data;
          printf("GPS FIXED: Lat: %f, Lon: %f\n", current_gps.lat,
                 current_gps.lon);
        } else {
          // This will help you see if the parser is at least seeing the strings
          if (strstr(gps_buffer, "RMC")) {
            printf("Found RMC string, but parser rejected it (Invalid Fix).\n");
          }
        }
        gps_idx = 0;
      } else {
        gps_buffer[gps_idx++] = c;
      }
    }

    // 2. Check if it's time to send the APRS beacon
    uint32_t current_time = to_ms_since_boot(get_absolute_time());
    if (current_time - last_tx_time >= tx_interval_ms) {

      if (current_gps.valid) {
        printf("10 minutes passed! Sending GPS fix...\n");

        // 1. Physically disable the GPS UART interrupts/peripheral
        uart_deinit(GPS_UART);

        gpio_put(PTT_GPXX_PIN, false);
        sleep_ms(PTT_DELAY_BEFORE_TX_IN_MSEC);

        // Send an APRS test message
        aprs_pico_sendAPRS(audio_buffer_pool,
                           "OE6UKN-12",                 // Source call sign
                           "APPIPI",                    // Destination call sign
                           "WIDE1-1",                   // APRS path #1
                           "WIDE2-2",                   // APRS path #2
                           "Pico Tracker by eleccoder", // Text
                           current_gps.lat, current_gps.lon,
                           0,     // Altitude  (in m)
                           '/',   // APRS symbol table: Primary
                           '>',   // APRS symbol code:  Car
                           128u); // Volume    (0 ... 256)

        gpio_put(PTT_GPXX_PIN, true);

        // 3. Re-enable the GPS UART
        uart_init(GPS_UART, GPS_BAUD);
        gpio_set_function(GPS_TX_PIN, GPIO_FUNC_UART);
        gpio_set_function(GPS_RX_PIN, GPIO_FUNC_UART);

        // 2. Clear the UART buffer of all the "garbage" that arrived during TX
        while (uart_is_readable(GPS_UART)) {
          uart_getc(GPS_UART);
        }
        gps_idx = 0; // Reset buffer index
        last_tx_time = current_time;
      } else {
        printf("Time to send, but no GPS fix yet. Waiting...\n");
        // Optional: Send anyway with last known location or wait 30s
      }
    }
    sleep_ms(1);
  }

  return 0;
}