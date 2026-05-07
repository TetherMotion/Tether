/**
 * @file example_01_basic_encoding.cpp
 * @brief Example: Basic SLIP encoding and decoding
 * 
 * Demonstrates how to use the Buffer API for simple SLIP encoding/decoding.
 * This is the most straightforward use case.
 */

#include "SLIPStream/Buffer.hpp"
#include <cstdio>
#include <cstring>
#include <vector>

using namespace SLIPStream;

int main() {
    printf("============================================================\n");
    printf("Example 1: Basic SLIP Encoding and Decoding\n");
    printf("============================================================\n\n");
    
    // ========================================================================
    // Example 1: Encode simple data without special bytes
    // ========================================================================
    printf("Example 1.1: Simple data (no escaping needed)\n");
    printf("------------------------------------------------------------\n");
    
    const uint8_t simple_data[] = {0x01, 0x02, 0x03, 0x04, 0x05};
    printf("Original data: ");
    for (auto b : simple_data) printf("%02X ", b);
    printf("\n");
    
    size_t encoded_len = encoded_length(simple_data, sizeof(simple_data));
    printf("Encoded length will be: %zu bytes\n", encoded_len);
    
    std::vector<uint8_t> encoded(encoded_len);
    size_t written = encode_packet(simple_data, sizeof(simple_data), 
                                   encoded.data(), encoded.size());
    
    printf("Encoded data:  ");
    for (size_t i = 0; i < written; i++) printf("%02X ", encoded[i]);
    printf("\n\n");
    
    // ========================================================================
    // Example 2: Encode data with special bytes (requires escaping)
    // ========================================================================
    printf("Example 1.2: Data with special bytes (needs escaping)\n");
    printf("------------------------------------------------------------\n");
    
    const uint8_t data_with_special[] = {0x01, END, 0x02, ESC, 0x03};
    printf("Original data with END (0xC0) and ESC (0xDB) bytes: ");
    for (auto b : data_with_special) printf("%02X ", b);
    printf("\n");
    printf("  0xC0 = END marker (frame terminator)\n");
    printf("  0xDB = ESC marker (escape character)\n");
    
    size_t escape_len = encoded_length(data_with_special, sizeof(data_with_special));
    std::vector<uint8_t> escaped(escape_len);
    encode_packet(data_with_special, sizeof(data_with_special), 
                  escaped.data(), escaped.size());
    
    printf("Escaped output: ");
    for (size_t i = 0; i < escaped_len; i++) printf("%02X ", escaped[i]);
    printf("\n");
    printf("  0xC0 → DB DC (ESC + ESCEND)\n");
    printf("  0xDB → DB DD (ESC + ESCESC)\n\n");
    
    // ========================================================================
    // Example 3: Decode the encoded data back
    // ========================================================================
    printf("Example 1.3: Decoding back to original\n");
    printf("------------------------------------------------------------\n");
    
    size_t decoded_len = decoded_length(escaped.data(), escaped.size());
    printf("Decoded length will be: %zu bytes\n", decoded_len);
    
    std::vector<uint8_t> decoded(decoded_len);
    size_t decoded_written = decode_packet(escaped.data(), escaped.size(),
                                          decoded.data(), decoded.size());
    
    printf("Decoded data:  ");
    for (size_t i = 0; i < decoded_written; i++) printf("%02X ", decoded[i]);
    printf("\n");
    
    // Verify round-trip
    if (std::memcmp(data_with_special, decoded.data(), sizeof(data_with_special)) == 0) {
        printf("✓ Round-trip successful: Original matches decoded!\n\n");
    } else {
        printf("✗ Error: Decoded data doesn't match original!\n\n");
        return 1;
    }
    
    // ========================================================================
    // Example 4: Error handling - buffer too small
    // ========================================================================
    printf("Example 1.4: Error handling\n");
    printf("------------------------------------------------------------\n");
    
    uint8_t small_buffer[2];
    const uint8_t test_data[] = {0x01, 0x02, 0x03};
    
    printf("Trying to encode into buffer that's too small...\n");
    size_t result = encode_packet(test_data, sizeof(test_data), 
                                  small_buffer, sizeof(small_buffer));
    
    if (result == ENCODE_ERROR) {
        printf("✓ Correctly returned ENCODE_ERROR (SIZE_MAX)\n\n");
    }
    
    printf("============================================================\n");
    printf("Example completed successfully!\n");
    return 0;
}
