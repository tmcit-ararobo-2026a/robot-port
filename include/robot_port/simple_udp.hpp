// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: MIT
#pragma once

#include <netinet/in.h>

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

namespace robot_port {
/** @brief IPv4 endpoint with host-order port. */
struct Endpoint {
    std::string address;
    uint16_t port{};
    /** @brief Compare full source endpoint, including port. */
    bool operator==(const Endpoint &) const = default;
};

/** @brief A receive attempt, distinguishing empty datagrams and no data. */
struct DatagramResult {
    enum class Status { NO_DATA, RECEIVED, ERROR };
    Status status{Status::NO_DATA};
    std::size_t size{};
    Endpoint source{};
    bool truncated{};
    int error_number{};
};

/** @brief Nonblocking, exclusively bound IPv4 UDP socket with RAII ownership. */
class UDPChannel
{
public:
    /** @brief Bind once; fail immediately on invalid address or occupied port. */
    UDPChannel(const Endpoint &local, const Endpoint &peer);
    /** @brief Close the owned descriptor, including exception paths. */
    ~UDPChannel();
    UDPChannel(const UDPChannel &)            = delete;
    UDPChannel &operator=(const UDPChannel &) = delete;
    /** @brief Send a complete datagram to the configured peer. */
    bool send(std::span<const uint8_t> bytes) const;
    /** @brief Send to an explicit destination, not the configured peer. */
    bool send_to(std::span<const uint8_t> bytes, const Endpoint &destination) const;
    /** @brief Receive one datagram and expose original length and truncation. */
    DatagramResult receive(std::span<uint8_t> buffer) const;
    /** @brief Return the actual bound port, including ephemeral test ports. */
    Endpoint local_endpoint() const;

private:
    int socket_{-1};
    sockaddr_in peer_address_{};
};
}  // namespace robot_port
