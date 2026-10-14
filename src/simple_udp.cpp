// Copyright 2026 tmcit-ararobo-2026a
// SPDX-License-Identifier: MIT
#include "robot_port/simple_udp.hpp"

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <stdexcept>
#include <system_error>

namespace robot_port {
namespace {
/** @brief Convert an endpoint, rejecting malformed IPv4 literals. */
sockaddr_in socket_address(const Endpoint &endpoint)
{
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port   = htons(endpoint.port);
    if (inet_pton(AF_INET, endpoint.address.c_str(), &address.sin_addr) != 1) {
        throw std::invalid_argument("Invalid IPv4 endpoint");
    }
    return address;
}

/** @brief Convert a kernel address to a host-order endpoint. */
Endpoint endpoint_of(const sockaddr_in &address)
{
    char text[INET_ADDRSTRLEN]{};
    if (inet_ntop(AF_INET, &address.sin_addr, text, sizeof(text)) == nullptr) {
        throw std::system_error(errno, std::generic_category(), "inet_ntop");
    }
    return {text, ntohs(address.sin_port)};
}
}  // namespace

UDPChannel::UDPChannel(const Endpoint &local, const Endpoint &peer)
{
    const auto local_address = socket_address(local);
    peer_address_            = socket_address(peer);
    socket_                  = ::socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (socket_ < 0) {
        throw std::system_error(errno, std::generic_category(), "socket");
    }
    // Do not use SO_REUSEADDR: another process must not share this control port.
    if (::bind(socket_, reinterpret_cast<const sockaddr *>(&local_address), sizeof(local_address)) <
        0) {
        const int bind_error = errno;
        ::close(socket_);
        socket_ = -1;
        throw std::system_error(bind_error, std::generic_category(), "bind");
    }
}

UDPChannel::~UDPChannel()
{
    if (socket_ >= 0) {
        ::close(socket_);
    }
}

bool UDPChannel::send(std::span<const uint8_t> bytes) const
{
    const auto sent = sendto(
        socket_,
        bytes.data(),
        bytes.size(),
        0,
        reinterpret_cast<const sockaddr *>(&peer_address_),
        sizeof(peer_address_)
    );
    return sent >= 0 && static_cast<std::size_t>(sent) == bytes.size();
}

bool UDPChannel::send_to(std::span<const uint8_t> bytes, const Endpoint &destination) const
{
    const auto address = socket_address(destination);
    const auto sent    = sendto(
        socket_,
        bytes.data(),
        bytes.size(),
        0,
        reinterpret_cast<const sockaddr *>(&address),
        sizeof(address)
    );
    return sent >= 0 && static_cast<std::size_t>(sent) == bytes.size();
}

DatagramResult UDPChannel::receive(std::span<uint8_t> buffer) const
{
    sockaddr_in source{};
    iovec vector{buffer.data(), buffer.size()};
    msghdr message{};
    message.msg_name    = &source;
    message.msg_namelen = sizeof(source);
    message.msg_iov     = &vector;
    message.msg_iovlen  = 1;
    const auto received = recvmsg(socket_, &message, MSG_TRUNC);
    if (received < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
            return {};
        }
        return {DatagramResult::Status::ERROR, 0, {}, false, errno};
    }
    return {
        DatagramResult::Status::RECEIVED,
        static_cast<std::size_t>(received),
        endpoint_of(source),
        (message.msg_flags & MSG_TRUNC) != 0,
        0};
}

Endpoint UDPChannel::local_endpoint() const
{
    sockaddr_in address{};
    socklen_t length = sizeof(address);
    if (getsockname(socket_, reinterpret_cast<sockaddr *>(&address), &length) < 0) {
        throw std::system_error(errno, std::generic_category(), "getsockname");
    }
    return endpoint_of(address);
}
}  // namespace robot_port
