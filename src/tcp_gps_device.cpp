#include <netdb.h>
#include <netinet/in.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#include "tcp_gps_device.h"
#include <system_error>

namespace xbot {
    namespace driver {
        namespace gps {
            void TcpGpsDevice::set_host(std::string host) {
                host_ = host;
            }

            void TcpGpsDevice::set_port(std::string port) {
                port_ = port;
            }

            bool TcpGpsDevice::check_parameters() {
                if (host_.empty()) {
                    log("no host set, can't start", ERROR);
                    return false;
                }
                if (port_.empty()) {
                    log("no port set, can't start", ERROR);
                    return false;
                }
                return true;
            }

            bool TcpGpsDevice::is_open() {
                return sockfd_ >= 0;
            }

            bool TcpGpsDevice::open() {
                log("connecting to " + host_ + ":" + port_, INFO);

                struct addrinfo hints = {
                    .ai_family = AF_UNSPEC,
                    .ai_socktype = SOCK_STREAM,
                    .ai_protocol = IPPROTO_TCP,
                };
                struct addrinfo *addrs;
                int err = getaddrinfo(host_.c_str(), port_.c_str(), &hints, &addrs);
                if (err != 0) {
                    log("could not resolve hostname " + host_ + ": " + gai_strerror(err), ERROR);
                    return false;
                } else if (addrs == nullptr) {
                    log("could not resolve hostname " + host_ + ": no IP found", ERROR);
                    return false;
                }

                sockfd_ = socket(addrs->ai_family, addrs->ai_socktype, addrs->ai_protocol);
                if (sockfd_ == -1) {
                    log("socket creation failed: " + std::string(strerror(errno)), ERROR);
                    freeaddrinfo(addrs);
                    return false;
                }

                struct timeval timeout = {
                    .tv_sec = 10,
                };
                setsockopt(sockfd_, SOL_SOCKET, SO_SNDTIMEO, &timeout, sizeof(timeout));
                setsockopt(sockfd_, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

                if (connect(sockfd_, addrs->ai_addr, addrs->ai_addrlen) == -1) {
                    log("connection failed: " + std::string(strerror(errno)), ERROR);
                    close();
                    freeaddrinfo(addrs);
                    return false;
                }

                freeaddrinfo(addrs);

                return true;
            }

            void TcpGpsDevice::close() {
                ::close(sockfd_);
                sockfd_ = -1;
            }

            size_t TcpGpsDevice::read(std::vector<uint8_t> &buffer, size_t) {
                buffer.resize(1024);
                ssize_t bytes_read = ::read(sockfd_, buffer.data(), 1024);
                if (bytes_read == -1) {
                    throw std::system_error(errno, std::generic_category());
                } else if (bytes_read == 0) {
                    throw std::runtime_error("connection closed");
                }
                buffer.resize(bytes_read);
                return bytes_read;
            }

            size_t TcpGpsDevice::write(const uint8_t *data, size_t size) {
                ssize_t bytes_written = ::write(sockfd_, data, size);
                if (bytes_written == -1) {
                    throw std::system_error(errno, std::generic_category());
                }
                return bytes_written;
            }
        }
    }
}
