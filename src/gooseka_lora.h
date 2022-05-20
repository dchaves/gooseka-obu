#ifndef GOOSEKA_LORA_H
#define GOOSEKA_LORA_H

#include <SPI.h>
#include <LoRa.h>
#include "gooseka_helpers.h"

// LORA TRANSCEIVER PINS
#define LORA_SCK 5
#define LORA_MISO 19
#define LORA_MOSI 27
#define LORA_SS 18
#define LORA_RST 14
#define LORA_DIO0 26

// LORA FREQUENCY BAND
// 433E6 for Asia
// 866E6 for Europe
// 915E6 for North America
#define LORA_BAND 866E6
#define LORA_SPREADING_FACTOR 7
#define LORA_BANDWIDTH 250e3
#define LORA_CODING_RATE 5
#define LORA_PREAMBLE_LENGTH 8
#define LORA_TX_POWER 20

// LORA SYNCWORD
#define LORA_SYNCWORD 0xCA

#define RECEIVED_BUFFER_SIZE 128

void init_radio();
void send_via_radio(uint8_t* payload, size_t size);
int receive_radio_packet(uint8_t* buffer, int size);
int16_t radio_rssi();

#endif /* GOOSEKA_LORA_H */