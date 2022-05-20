#include "gooseka_lora.h"

uint8_t received_packet[RECEIVED_BUFFER_SIZE];
int received_size;

void init_radio() {
    // Set SPI LoRa pins
    SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_SS);

    // Setup LoRa transceiver module
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    if(!LoRa.begin(LORA_BAND)) {
        DEBUG_PRINTLN("LoRa initialization error");
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
    int i = 0;
    int packetSize = LoRa.parsePacket();
    if (packetSize == size) { // Correct message size. Receive
        while (LoRa.available() && (i < size)) {
            received_packet[i] = (uint8_t)LoRa.read();
            i++;
        }
        memcpy(buffer, received_packet, size);
        return size;
    } else { // Incorrect message size. Empty packet buffer
        while (LoRa.available()) {
            LoRa.read();
        }
        return 0;
    }
}

int16_t radio_rssi() {
    return LoRa.packetRssi();
}