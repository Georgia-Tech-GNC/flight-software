#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>


// Precompute all hashes for 0xe7
uint8_t crctable[256] = {0, 231, 41, 206, 82, 181, 123, 156, 164, 67, 141, 106, 246, 17,
 223, 56, 175, 72, 134, 97, 253, 26, 212, 51, 11, 236, 34, 197, 89, 190, 112, 151, 185, 94, 
 144, 119, 235, 12, 194, 37, 29, 250, 52, 211, 79, 168, 102, 129, 22, 241, 63, 216, 68, 163, 
 109, 138, 178, 85, 155, 124, 224, 7, 201, 46, 149, 114, 188, 91, 199, 32, 238, 9, 49, 214, 24, 
 255, 99, 132, 74, 173, 58, 221, 19, 244, 104, 143, 65, 166, 158, 121, 183, 80, 204, 43, 229, 2, 
 44, 203, 5, 226, 126, 153, 87, 176, 136, 111, 161, 70, 218, 61, 243, 20, 131, 100, 170, 77, 209, 
 54, 248, 31, 39, 192, 14, 233, 117, 146, 92, 187, 205, 42, 228, 3, 159, 120, 182, 81, 105, 142, 64, 
 167, 59, 220, 18, 245, 98, 133, 75, 172, 48, 215, 25, 254, 198, 33, 239, 8, 148, 115, 189, 90, 116, 
 147, 93, 186, 38, 193, 15, 232, 208, 55, 249, 30, 130, 101, 171, 76, 219, 60, 242, 21, 137, 110, 160, 71, 
 127, 152, 86, 177, 45, 202, 4, 227, 88, 191, 113, 150, 10, 237, 35, 196, 252, 27, 213, 50, 174, 73, 135, 96, 
 247, 16, 222, 57, 165, 66, 140, 107, 83, 180, 122, 157, 1, 230, 40, 207, 225, 6, 200, 47, 179, 84, 154, 125, 69, 
 162, 108, 139, 23, 240, 62, 217, 78, 169, 103, 128, 28, 251, 53, 210, 234, 13, 195, 36, 184, 95, 145, 118
};

/** 
 * Returns the 8-bit CRC hash of this data
 * 
 * @param rawData   pointer to the data that the hash should be calculated for
 * @param dataSize  size of the data in bytes
 * 
 * @return the CRC8 hash of the data
 */
uint8_t calculate_crc8_hash(const uint8_t *raw_data, const size_t data_size) {
    uint8_t crc = 0;
    for (int i = 0; i < data_size; i++)
    {
        // XOR-in next input byte and get current CRC value = remainder 
        crc = crctable[raw_data[i] ^ crc];
    }

    return crc;
}

/**
 * Returns true if the crc8 hash is valid for this data. It assumes that the 8-bit crc hash immediately follows the data
 * 
 * @param rawData   pointer to the data that the hash should be verified for (last byte should be the hash)
 * @param dataSize  size of the data in bytes (including the hash byte)
 * 
 * Note that size includes the hash byte. In other words, if you want to verify the hash of n bytes, pass in 
 * an n+1 byte array where the first n bytes are the data and the n+1-th byte is the hash. 
 * data_size should also equal n+1.
 * 
 * @return true if the hash was valid, and false otherwise
 */
bool verify_crc8_hash(const uint8_t *data, const size_t data_size) {
    return 0 == calculate_crc8_hash(data, data_size);
}