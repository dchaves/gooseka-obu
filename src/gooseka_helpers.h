#ifndef GOOSEKA_HELPERS_H
#define GOOSEKA_HELPERS_H

#include <HardwareSerial.h>
#include "gooseka_structs.h"
#include "gooseka_defs.h"

#define ENABLE_DEBUG true // Enable or disable USB Serial console

#if ENABLE_DEBUG // Serial printing enabled
#define DEBUG_BEGIN(BAUDS) Serial.begin(BAUDS)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_TELEMETRY(S,T) print_telemetry(S,T)
#else // Serial printing disabled
#define DEBUG_BEGIN(BAUDS) 
#define DEBUG_PRINTLN(...) 
#define DEBUG_PRINT(...) 
#define DEBUG_TELEMETRY(S,T)
#endif /* ENABLE_DEBUG */

typedef struct {
    uint8_t received_bytes;
    uint8_t buffer[SERIAL_BUFFER_SIZE];
} serial_buffer_t;

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed);
uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen);
void print_telemetry(HardwareSerial *serial, ESC_telemetry_t *telemetry);
bool read_telemetry(HardwareSerial* serial, serial_buffer_t* serial_buffer, ESC_oneside_telemetry_t* telemetry);
uint32_t time_since(uint32_t reference_time);

#endif /* GOOSEKA_HELPERS_H */
