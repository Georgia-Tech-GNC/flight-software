#ifndef PACKET_ENCODE
#define PACKET_ENCODE

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/** 
 * Takes any raw data (up to 253 bytes), and converts into a packet (complete with byte stuffing and a checksum)
 * Packet format: 0x00 [messageID - 1 byte] [payload size - 1 byte] [COBS] [payload] [checksum(CRC-8) - 1 byte]
 * 
 * @param rawData           the raw payload data as an array of uint8_t
 * @param packetizedData    a pointer to a uint8_t array where the generated packet should be stored
 * @param dataSize          the length of the raw data in bytes
 * @param messageID         a one-byte message id that should be associated with this data (must not be 0)
 * @return                  The length of the packetized data in bytes
 * 
 * packetizedData should be at least 5 bytes longer then the rawData.
 */
int generate_packet(const uint8_t *rawData, const uint8_t dataSize, uint8_t *packetizedData, const uint8_t messageID);

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
bool verify_packet(uint8_t *rawData, size_t dataSize);

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
uint8_t extract_packet(uint8_t *rawData, const size_t dataSize, uint8_t *extractedData);


/** Process the incoming byte and add it to a buffer
 * 
 * @param inputted_byte             the recieved byte
 * @param buffer                    the buffer that stores recieved bytes
 * @param current_buffer_size       the size of the buffer currently in use
 * @return                          The new buffer size. If the value is negative, then the
 *                                  complete message has been recieved (the length is the absolute value of the returned value)
 */
int process_incoming_byte(const uint8_t inputted_byte, uint8_t *buffer, const int current_buffer_size);

#endif