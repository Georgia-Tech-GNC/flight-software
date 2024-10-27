#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "crc_hash.h"

/** 
 * Applies the COBS algorithm to an input rawData.
 * This method removes all instances of the 0x00 byte by stuffing a linked list of bytes into the packet to represent the missing bytes. 
 * This last instance of the 0x00 byte will point to the index after the array
 * Note that the raw data may NOT be more than 254 bytes long. 
 * The encoded data will be exactly one byte longer than the raw data (the first byte is the COBS byte).
 * 
 * @param   rawData                 A pointer to the start of a uint8_t[] representing the data to encode
 * @param   encodedData             A pointer to the start of a uint8_t[]. This array will be overwritten with the encoded data
 * @param   dataSize                The length of the raw data (in bytes)
 * @param   encodedDataStartIndex   Represents the index at which this method should begin storing the encoded data in the encodedData array
 * @return                          The index of the final stuffed bit in the encodedData array
 */
int cobs_encode(const uint8_t *rawData, const uint8_t dataSize, uint8_t *encodedData, const int encodedDataStartIndex) {
    int lastStuffedByteIndex = 0;
    encodedData[encodedDataStartIndex] = dataSize + 1;

    for (int i = 1; i <= dataSize; i++) {
        if (rawData[i - 1] == 0x00) {
            // Apply COBS algorithm and set index from previous COBS byte to point here
            encodedData[encodedDataStartIndex + lastStuffedByteIndex] = i - lastStuffedByteIndex;
            lastStuffedByteIndex = i;
            encodedData[encodedDataStartIndex + i] = dataSize + 1 - i;
        } else {
            encodedData[encodedDataStartIndex + i] = rawData[i - 1];
        }
    }

    return lastStuffedByteIndex + encodedDataStartIndex;
}


int generate_packet(const uint8_t *rawData, const uint8_t dataSize, uint8_t *packetizedData, const uint8_t messageID) {
    packetizedData[0] = 0x00;                               // Byte 0: 0x00
    packetizedData[1] = messageID;                          // Byte 1: message id
    packetizedData[2] = dataSize;                           // Byte 2: payload size
    int cobs_overwrite_index = cobs_encode(rawData, dataSize, packetizedData, 3);  
                                                            // Byte 3 COBS byte         
                                                            // Bytes 4 - dataSize+3: payload

    // Calculate the CRC-8 checksum
    uint8_t crc = calculate_crc8_hash(packetizedData, dataSize + 4);
                                                   
    if (crc == 0x00) {                                      // Byte dataSize+4: checksum
        packetizedData[dataSize + 4] = 1;
    } else {
        packetizedData[dataSize + 4] = crc;
        packetizedData[cobs_overwrite_index]++;     // cobs encoding automatically assumed that crc == 0, so we correct the assumption by
                                                    // incrementing the last jump by one
    }

    return dataSize + 5;
}


bool verify_packet(uint8_t *rawData, const size_t dataSize) {
    if (dataSize < 5) return false;
    // First either de-stuff the crc_hash byte if needed or move back the last cobs pointer
    
    int next_cobs_byte = 3;
    while (next_cobs_byte < dataSize) {
        int increment = rawData[next_cobs_byte];
        if (increment == 0) return false; // Avoid infinite loops
        else if (next_cobs_byte + increment == dataSize - 1) {
            rawData[dataSize-1] = 0;
            break;
        } else if (next_cobs_byte + increment >= dataSize) {
            rawData[next_cobs_byte]--;
        }

        next_cobs_byte += increment;
    } 

    return verify_crc8_hash(rawData+1, dataSize-1);
}


uint8_t extract_packet(uint8_t *rawData, const size_t dataSize, uint8_t *extractedData) {
    uint8_t message_id = rawData[1];

    // First de-stuff the packet
    int next_cobs_byte = 3;
    while (next_cobs_byte < dataSize - 1) {
        int increment = rawData[next_cobs_byte];
        if (increment == 0) return 0;
        rawData[next_cobs_byte] = 0;

        next_cobs_byte += increment;
    }  

    memcpy(extractedData, rawData + 4, dataSize - 5);

    return message_id;
}

int process_incoming_byte(const uint8_t inputted_byte, uint8_t *buffer, const int current_buffer_size) {
    if (inputted_byte == 0) { // New message started
        buffer[0] = 0;
        return 1;
    }

    if (current_buffer_size == 0)  // No currently active message
        return 0;
    

    buffer[current_buffer_size] = inputted_byte;

    if (current_buffer_size >= 2) { // We know the size
        if (current_buffer_size + 1 >= buffer[2] + 5) {
            return -buffer[2] - 5;
        }
    }
    
    return current_buffer_size + 1;
}
