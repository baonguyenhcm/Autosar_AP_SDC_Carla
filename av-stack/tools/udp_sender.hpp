// tools/udp_sender.hpp
// Tiny header-only UDP datagram sender, portable across POSIX and Windows.
// Used by pipeline_demo (when built with -DAV_UDP) to stream ego pose to a
// live Unreal Engine viewer. Kept optional so the default demo stays a pure,
// dependency-free C++17 program.
#pragma once

#include <string>
#include <cstdint>

#if defined(_WIN32)
  #include <winsock2.h>
  #include <ws2tcpip.h>
  #pragma comment(lib, "ws2_32.lib")
  using av_socket_t = SOCKET;
  static const av_socket_t AV_BAD_SOCKET = INVALID_SOCKET;
#else
  #include <sys/socket.h>
  #include <netinet/in.h>
  #include <arpa/inet.h>
  #include <unistd.h>
  using av_socket_t = int;
  static const av_socket_t AV_BAD_SOCKET = -1;
#endif

class UdpSender {
public:
  UdpSender(const std::string& ip = "127.0.0.1", std::uint16_t port = 9999) {
#if defined(_WIN32)
    WSADATA wsa;
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0) { return; }
    wsa_ready_ = true;
#endif
    sock_ = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_ == AV_BAD_SOCKET) { return; }
    dst_.sin_family = AF_INET;
    dst_.sin_port = htons(port);
    inet_pton(AF_INET, ip.c_str(), &dst_.sin_addr);
  }

  ~UdpSender() {
    if (sock_ != AV_BAD_SOCKET) {
#if defined(_WIN32)
      closesocket(sock_);
#else
      ::close(sock_);
#endif
    }
#if defined(_WIN32)
    if (wsa_ready_) { WSACleanup(); }
#endif
  }

  bool ok() const { return sock_ != AV_BAD_SOCKET; }

  // Send one datagram. The trailing newline (if any) is kept; the receiver
  // treats each datagram as one line.
  void send(const std::string& msg) {
    if (sock_ == AV_BAD_SOCKET) { return; }
    ::sendto(sock_, msg.data(), static_cast<int>(msg.size()), 0,
             reinterpret_cast<const sockaddr*>(&dst_), sizeof(dst_));
  }

  UdpSender(const UdpSender&) = delete;
  UdpSender& operator=(const UdpSender&) = delete;

private:
  av_socket_t sock_ = AV_BAD_SOCKET;
  sockaddr_in dst_{};
#if defined(_WIN32)
  bool wsa_ready_ = false;
#endif
};
