#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>

#include "crc_hash.h"

/**
 * Iterates through the inputted array and replaces all zeroes.
 * Note that the first element in this array will be the COBS byte and will always be overwritten.
 * 
 * @param data_to_encode    the array of data to encode, including the initial COBS byte (will be modified in place)
 * @param data_size         length of the array (including the first COBS byte)
 * 
 * It is guaranteed that the last replaced 0 will point to one after the end of the array
 * @return the index of the last modified byte.
 */
int cobs_encode(uint8_t *data_to_encode, const uint8_t data_size) {
    if (data_size == 0) return 0;
    int last_stuffed_byte = 0;
    data_to_encode[0] = data_size;

    for (int i = 1; i < data_size; i++) {
        if (data_to_encode[i] == 0x00) { // Add to internal linked list
            data_to_encode[last_stuffed_byte] = i - last_stuffed_byte;
            last_stuffed_byte = i;
            data_to_encode[i] = data_size - i;
        }
    }

    return last_stuffed_byte;
}

/** 
 * Takes any raw data (up to 252 bytes), and converts into a packet (complete with byte stuffing and a checksum)
 * Packet format: 0x00 [messageID - 1 byte] [COBS] [payload size - 1 byte] [payload] [checksum(CRC-8) - 1 byte]
 * 
 * @param rawData           the raw payload data as an array of uint8_t
 * @param packetizedData    a pointer to a uint8_t array where the generated packet should be stored
 * @param dataSize          the length of the raw data in bytes
 * @param messageID         a one-byte message id that should be associated with this data (must not be 0)
 * @return                  The length of the packetized data in bytes
 * 
 * packetizedData should be at least 5 bytes longer then the rawData.
 */
int generate_packet(const uint8_t *rawData, const uint8_t dataSize, uint8_t *packetizedData, const uint8_t messageID) {
    packetizedData[0] = 0x00;                               // Byte 1: 0x00
    packetizedData[1] = messageID;                          // Byte 2: message id
    packetizedData[3] = dataSize;                           // Byte 4: payload size (skip one for the COBS byte)
    memcpy(packetizedData+4, rawData, dataSize);            // Copy payload starting at byte 5
    int cobs_overwrite_index = cobs_encode(packetizedData + 2, dataSize + 2);  // Apply COBS encoding

    // Calculate the CRC-8 checksum
    uint8_t crc = calculate_crc8_hash(packetizedData, dataSize + 4);
                                                   
    if (crc == 0x00) {                                      // Byte dataSize+4: checksum
        packetizedData[dataSize + 4] = 1;
    } else {
        packetizedData[dataSize + 4] = crc;
        packetizedData[cobs_overwrite_index + 2]++; // cobs encoding automatically assumed that crc == 0, so we correct the assumption by
                                                    // incrementing the last jump by one
    }

    return dataSize + 5;
}


/** 
 * Returns true if the crc hash for this packet matches the data. The five overhead bytes should be included in rawData
 * This method may modify the final byte in the packet (the crc hash)
 * 
 * @param rawData   the packet to verify
 * @param dataSize  the size of the packet (including the five overhead bytes)
 * 
 * @return true if the packet could be verified, and false otherwise
 * 
 * THE LEADING 0 BYTE SHOULD BE INCLUDED IN THE DATA
 */
bool verify_packet(uint8_t *rawData, const size_t dataSize) {
    if (dataSize < 5) return false;
    // First either de-stuff the crc_hash byte if needed or move back the last cobs pointer

    if (rawData[0] != 0) return false; // Leading byte was not 0.
    
    int next_cobs_byte = 2;
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

    return verify_crc8_hash(rawData, dataSize);
}

/** Extracts the stored data from a packet and copies it to the extractedData location 
 * 
 * This method assumes that the packet was verified. Please use verify_packet first.
 * 
 * @param rawData   the packet to extract
 * @param dataSize  the size of the packet (including the 5 overhead bytes)
 * @param extractedData the buffer where the extracted data should be stored
 * 
 * This method will store exactly dataSize-5 bytes in the extractedData buffer
 * 
 * @return      the message id, or 0 if the message was corrupted
 */
uint8_t extract_packet(uint8_t *rawData, const size_t dataSize, uint8_t *extractedData) {
    if (dataSize < 5) return 0;

    uint8_t message_id = rawData[1];

    if (message_id == 0) return 0;

    // First de-stuff the packet
    int next_cobs_byte = 2;
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

    if (current_buffer_size >= 3) { // We know the size
        int msg_size = (buffer[2] == 1)?5:(buffer[3] + 5);
        if (current_buffer_size + 1 >= msg_size) {
            return -msg_size;
        }
    }
    
    return current_buffer_size + 1;
}