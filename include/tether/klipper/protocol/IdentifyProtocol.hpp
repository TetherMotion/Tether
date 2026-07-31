/**
 * @file IdentifyProtocol.hpp
 * @brief `identify` / `identify_response` handshake to download the data dict.
 *
 * @details
 * Before the data dictionary is available, the host and device communicate
 * using two hard-coded messages:
 *
 *   identify_response  (MCU -> host, msgid 0):  "identify_response offset=%u data=%.*s"
 *   identify            (host -> MCU, msgid 1):  "identify offset=%u count=%c"
 *
 * The host requests chunks of the (compressed) data dictionary starting at
 * @p offset with up to @p count bytes; the device responds with the chunk.
 * Download completes when a response carries an empty data payload.
 *
 * This header provides:
 *   - IdentifyServer (device side): serves chunks from a stored wire blob.
 *   - IdentifyClient (host side):   requests chunks and assembles the blob.
 */

#pragma once

#include "tether/klipper/protocol/Constants.hpp"
#include "tether/klipper/protocol/Vlq.hpp"
#include "tether/klipper/protocol/ParameterFormat.hpp"
#include "tether/klipper/protocol/DataDictionary.hpp"

#include <cstdint>
#include <vector>
#include <span>
#include <optional>
#include <functional>

namespace tether::klipper::protocol {

/**
 * @brief Device-side identify server: serves chunks of the compressed data
 *        dictionary blob to host `identify` requests.
 */
class IdentifyServer {
public:
    /// @brief Construct with the wire-format (compressed) dictionary blob.
    explicit IdentifyServer(std::vector<uint8_t> wireBlob)
        : blob_(std::move(wireBlob)) {}

    /// @return Total size of the dictionary blob in bytes.
    size_t size() const { return blob_.size(); }

    /**
     * @brief Build an `identify_response` content payload for a request.
     * @param offset Requested byte offset into the blob.
     * @param count  Requested maximum chunk size.
     * @return Content bytes encoding the response message (msgid 0 + params).
     */
    std::vector<uint8_t> buildResponseContent(uint32_t offset, uint8_t count) const;

private:
    std::vector<uint8_t> blob_;
};

/**
 * @brief Host-side identify client: requests chunks and assembles the blob.
 *
 * The caller drives the request/response exchange (e.g. via a transport);
 * this class tracks the next offset to request and accumulates received
 * chunks.
 */
class IdentifyClient {
public:
    IdentifyClient() = default;

    /// @brief Build the content for an `identify` request for the next chunk.
    /// @param count Maximum chunk size to request.
    std::vector<uint8_t> buildRequestContent(uint8_t count = kDefaultIdentifyChunkSize) const;

    /// @return The next offset that will be requested.
    uint32_t nextOffset() const { return static_cast<uint32_t>(received_.size()); }

    /// @brief Consume an `identify_response` content payload.
    /// @return true if the response offset matched the expected next offset.
    bool consumeResponseContent(std::span<const uint8_t> content);

    /// @return true once an empty-data response has been received (download done).
    bool complete() const { return complete_; }

    /// @return The assembled (compressed) wire blob.
    const std::vector<uint8_t>& wireBlob() const { return received_; }

    /**
     * @brief Decode the assembled wire blob into a DataDictionary.
     * @return The parsed dictionary, or std::nullopt on failure.
     */
    std::optional<DataDictionary> decodeDictionary() const;

private:
    std::vector<uint8_t> received_;
    bool complete_ = false;
};

} // namespace tether::klipper::protocol
