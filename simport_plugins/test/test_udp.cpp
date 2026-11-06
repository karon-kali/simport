// Copyright 2026 simport contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "simport_plugins/udp.hpp"

using namespace std::chrono_literals;

namespace simport::plugins
{
namespace
{

/// A plain UDP socket playing the driver.
struct Peer
{
  int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  std::uint16_t port = 0;

  explicit Peer(const char * host = "127.0.0.1")
  {
    sockaddr_in a{};
    a.sin_family = AF_INET;
    ::inet_pton(AF_INET, host, &a.sin_addr);
    ::bind(fd, reinterpret_cast<sockaddr *>(&a), sizeof(a));
    socklen_t len = sizeof(a);
    ::getsockname(fd, reinterpret_cast<sockaddr *>(&a), &len);
    port = ntohs(a.sin_port);
  }
  ~Peer() {::close(fd);}

  void send_to(const char * host, std::uint16_t to, const Bytes & data) const
  {
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(to);
    ::inet_pton(AF_INET, host, &a.sin_addr);
    ::sendto(fd, data.data(), data.size(), 0, reinterpret_cast<sockaddr *>(&a), sizeof(a));
  }

  /// Receive one datagram and its sender within 1 s.
  bool recv(Bytes & data, std::string & from) const
  {
    pollfd p{fd, POLLIN, 0};
    if (::poll(&p, 1, 1000) <= 0) {
      return false;
    }
    std::uint8_t buf[2048];
    sockaddr_in a{};
    socklen_t len = sizeof(a);
    const ssize_t n = ::recvfrom(fd, buf, sizeof(buf), 0, reinterpret_cast<sockaddr *>(&a), &len);
    data.assign(buf, buf + n);
    char host[INET_ADDRSTRLEN];
    ::inet_ntop(AF_INET, &a.sin_addr, host, sizeof(host));
    from = std::string(host) + ":" + std::to_string(ntohs(a.sin_port));
    return true;
  }
};

Context ctx()
{
  return Context("dev", std::make_shared<NullLogger>(), std::make_shared<SystemClock>());
}

TEST(Udp, SendsFromDeviceAddressToDestination)
{
  Peer driver;
  Udp udp;
  Context c = ctx();
  // 127.0.0.2: a distinct "device IP" without any network setup.
  udp.configure(Config::from_string(
      "udp: 127.0.0.2:0\nto: {host: 127.0.0.1, port: " +
      std::to_string(driver.port) + "}\n"), c);
  udp.open();
  EXPECT_FALSE(udp.caps().stream);
  ASSERT_TRUE(udp.write(Frame{{1, 2, 3}, {}, {}}));
  Bytes data;
  std::string from;
  ASSERT_TRUE(driver.recv(data, from));
  EXPECT_EQ(data, (Bytes{1, 2, 3}));
  EXPECT_EQ(from, "127.0.0.2:" + std::to_string(udp.local_port()));
  EXPECT_EQ(udp.stats().frames_tx, 1u);
  udp.close();
  EXPECT_FALSE(udp.write(Frame{{1}, {}, {}}));
}

TEST(Udp, ReceivesWithSenderAndRepliesToIt)
{
  Udp udp;
  Context c = ctx();
  udp.configure(Config::from_string("udp: 127.0.0.1:0\n"), c);  // replies only
  std::mutex m;
  std::vector<std::pair<Bytes, FrameMeta>> got;
  udp.set_receive_callback(
    [&](ByteView d, const FrameMeta & meta, Timestamp) {
      std::lock_guard<std::mutex> lock(m);
      got.emplace_back(Bytes(d.begin(), d.end()), meta);
    });
  udp.open();

  Peer driver;
  driver.send_to("127.0.0.1", udp.local_port(), {0x10, 0x20});
  for (int i = 0; i < 100; ++i) {
    {
      std::lock_guard<std::mutex> lock(m);
      if (!got.empty()) {break;}
    }
    std::this_thread::sleep_for(10ms);
  }
  std::lock_guard<std::mutex> lock(m);
  ASSERT_EQ(got.size(), 1u);
  EXPECT_EQ(got[0].first, (Bytes{0x10, 0x20}));
  ASSERT_TRUE(got[0].second.endpoint.has_value());
  EXPECT_EQ(got[0].second.endpoint->port, driver.port);

  EXPECT_FALSE(udp.write(Frame{{9}, {}, {}}));  // no destination, no requester
  ASSERT_TRUE(udp.write(Frame{{0xAC}, got[0].second, {}}));
  Bytes reply;
  std::string from;
  ASSERT_TRUE(driver.recv(reply, from));
  EXPECT_EQ(reply, (Bytes{0xAC}));
}

TEST(Udp, LearnsDestinationFromSender)
{
  Udp udp;
  Context c = ctx();
  udp.configure(Config::from_string("udp: 127.0.0.1:0\nto: sender\n"), c);
  udp.open();
  EXPECT_FALSE(udp.write(Frame{{1}, {}, {}}));  // nobody has talked to us yet
  Peer driver;
  driver.send_to("127.0.0.1", udp.local_port(), {0x10});
  bool sent = false;
  for (int i = 0; i < 100 && !sent; ++i) {
    std::this_thread::sleep_for(10ms);
    sent = udp.write(Frame{{0x42}, {}, {}});
  }
  ASSERT_TRUE(sent);
  Bytes data;
  std::string from;
  ASSERT_TRUE(driver.recv(data, from));
  EXPECT_EQ(data, (Bytes{0x42}));
}

TEST(Udp, EndpointParsing)
{
  EXPECT_EQ(parse_endpoint(Config::from_string("\"localhost:80\"")),
    (Endpoint{"localhost", 80}));
  EXPECT_THROW(parse_endpoint(Config::from_string("\"1.2.3.4\"")), ConfigError);
  EXPECT_THROW(parse_endpoint(Config::from_string("\"1.2.3.4:70000\"")), ConfigError);
  EXPECT_THROW(parse_endpoint(Config::from_string("{host: no.such.host.invalid, port: 1}")),
    ConfigError);
}

TEST(Udp, BindFailureIsReported)
{
  Peer taken("127.0.0.1");
  Udp udp;
  Context c = ctx();
  // SO_REUSEADDR does not allow two UDP sockets on one unicast address unless both set it.
  udp.configure(Config::from_string(
      "udp: 127.0.0.1:" + std::to_string(taken.port) + "\n"), c);
  EXPECT_THROW(udp.open(), TransportError);
}

}  // namespace
}  // namespace simport::plugins
