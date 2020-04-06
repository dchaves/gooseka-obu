#ifndef ESP32_RADIO_LORA_H
#define ESP32_RADIO_LORA_H

#include <SPI.h>
#include <LoRa.h>

#include "gooseka_defs.h"
#include "gooseka_structs.h"
#include "gooseka_helpers.h"

void init_radio() {
    // Set SPI LoRa pins
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

    // Setup LoRa transceiver module
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    if(!LoRa.begin(LORA_BAND)) {
        DEBUG_PRINTLN("LoRa init error.");
        while(1);
    }

    LoRa.setCodingRate4(LORA_CODING_RATE);
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.setSyncWord(LORA_SYNCWORD);
}

void send_via_radio(uint8_t* payload, size_t size) {
    LoRa.beginPacket();
    LoRa.write(payload, size); 
    if(LoRa.endPacket() == 1) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    }
}

int receive_radio_packet(uint8_t* buffer, int size) {
    uint8_t index = 0;

    int packetSize = LoRa.parsePacket();
    if (packetSize == size) {
        while (LoRa.available() && index < size) {
            buffer[index] = LoRa.read();
            index++;
        }
    }

    return packetSize;
}

int16_t radio_rssi() {
    return LoRa.packetRssi();
}

#endif // ESP32_RADIO_LORA_H