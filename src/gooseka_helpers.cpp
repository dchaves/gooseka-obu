#include "gooseka_helpers.h"


uint8_t update_crc8(uint8_t crc, uint8_t crc_seed) {
    uint8_t crc_u, i;
    crc_u = crc;
    crc_u ^= crc_seed;
    for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
    return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen) {
    uint8_t crc = 0, i;
    for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
    return (crc);
}

void print_telemetry(HardwareSerial *serial, ESC_telemetry_t *telemetry) {
    serial->print("LEFT,");
    serial->print(telemetry->left.timestamp); 
    serial->print(","); 
    serial->print(telemetry->left.temperature / 100.0); 
    serial->print(",");
    serial->print(telemetry->left.voltage / 100.0); 
    serial->print(","); 
    serial->print(telemetry->left.current / 10.0); 
    serial->print(","); 
    serial->print(telemetry->left.power); 
    serial->print(","); 
    serial->println(telemetry->left.erpm * 100 / (MOTOR_POLES / 2)); 

    serial->print("RIGHT,");
    serial->print(telemetry->right.timestamp); 
    serial->print(","); 
    serial->print(telemetry->right.temperature / 100.0); 
    serial->print(",");
    serial->print(telemetry->right.voltage / 100.0); 
    serial->print(","); 
    serial->print(telemetry->right.current / 10.0); 
    serial->print(","); 
    serial->print(telemetry->right.power); 
    serial->print(","); 
    serial->println(telemetry->right.erpm * 100 / (MOTOR_POLES / 2)); 
}

bool read_telemetry(HardwareSerial* serial, serial_buffer_t* serial_buffer, ESC_oneside_telemetry_t* telemetry) {
    serial_buffer->buffer[serial_buffer->received_bytes] = serial->read();
    serial_buffer->received_bytes++;

    if(serial_buffer->received_bytes > 9){ // transmission completed
        serial_buffer->received_bytes = 0;
        uint8_t crc8 = get_crc8(serial_buffer->buffer, 9); // get the 8 bit CRC
        
        if(crc8 != serial_buffer->buffer[9]) { // CRC ERROR                
            // Empty Rx Serial of garbage telemtry
            while(serial->available()) {
                serial->read();
            }
            return false; // transmission failure 
        }

        // compute the received values
        telemetry->timestamp = millis();
        telemetry->temperature = serial_buffer->buffer[0]; // temperature
        telemetry->voltage = (serial_buffer->buffer[1]<<8) | serial_buffer->buffer[2]; // voltage
        telemetry->current = (serial_buffer->buffer[3]<<8) | serial_buffer->buffer[4]; // Current
        telemetry->power = (serial_buffer->buffer[5]<<8) | serial_buffer->buffer[6]; // used mA/h
        telemetry->erpm = (serial_buffer->buffer[7]<<8) | serial_buffer->buffer[8]; // eRpM *100

        return true;
    }
    return false;
}

uint32_t time_since(uint32_t reference_time) {
    return millis() - reference_time;
}