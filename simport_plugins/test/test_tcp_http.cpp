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
#include <filesystem>
#include <fstream>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "simport_core/input_channel.hpp"
#include "simport_plugins/framers.hpp"
#include "simport_plugins/http_codec.hpp"
#include "simport_plugins/tcp_server.hpp"

using namespace std::chrono_literals;

namespace simport::plugins
{
namespace
{

Context make_context()
{
  return Context("dev", std::make_shared<NullLogger>(), std::make_shared<SystemClock>());
}

/// A TCP client playing the driver.
struct Client
{
  int fd = ::socket(AF_INET, SOCK_STREAM, 0);

  explicit Client(std::uint16_t port)
  {
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_port = htons(port);
    ::inet_pton(AF_INET, "127.0.0.1", &a.sin_addr);
    EXPECT_EQ(::connect(fd, reinterpret_cast<sockaddr *>(&a), sizeof(a)), 0);
  }
  ~Client() {close();}
  void close()
  {
    if (fd >= 0) {
      ::close(fd);
      fd = -1;
    }
  }

  void send(const std::string & text) const {::send(fd, text.data(), text.size(), 0);}

  std::string receive(std::size_t n, std::chrono::milliseconds timeout = 2s) const
  {
    std::string out;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (out.size() < n && std::chrono::steady_clock::now() < deadline) {
      pollfd p{fd, POLLIN, 0};
      if (::poll(&p, 1, 20) > 0) {
        char buf[4096];
        const ssize_t got = ::recv(fd, buf, sizeof(buf), 0);
        if (got <= 0) {
          break;
        }
        out.append(buf, static_cast<std::size_t>(got));
      }
    }
    return out;
  }
};

struct Received
{
  std::mutex mutex;
  std::vector<std::pair<std::string, FrameMeta>> chunks;

  void add(ByteView data, const FrameMeta & meta)
  {
    std::lock_guard<std::mutex> lock(mutex);
    chunks.emplace_back(std::string(data.begin(), data.end()), meta);
  }
  std::size_t size()
  {
    std::lock_guard<std::mutex> lock(mutex);
    return chunks.size();
  }
};

template<typename Pred>
bool wait_for(Pred pred, std::chrono::milliseconds timeout = 2s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (!pred()) {
    if (std::chrono::steady_clock::now() > deadline) {
      return false;
    }
    std::this_thread::sleep_for(5ms);
  }
  return true;
}

Frame text_frame(const std::string & text, std::optional<Endpoint> to = std::nullopt)
{
  Frame f{Bytes(text.begin(), text.end()), {}, {}};
  f.meta.endpoint = std::move(to);
  return f;
}

TEST(TcpServer, RepliesToTheAskingClientAndStreamsToAll)
{
  Context ctx = make_context();
  TcpServer server;
  server.configure(Config::from_string("tcp: 127.0.0.1:0"), ctx);
  Received rx;
  server.set_receive_callback([&](ByteView d, const FrameMeta & m, Timestamp) {rx.add(d, m);});
  server.open();
  ASSERT_GT(server.local_port(), 0);
  EXPECT_FALSE(*server.stats().peer_connected);
  EXPECT_FALSE(server.write(text_frame("nobody")));

  Client a(server.local_port());
  Client b(server.local_port());
  ASSERT_TRUE(wait_for([&] {return *server.stats().peer_connected;}));
  a.send("from a");
  ASSERT_TRUE(wait_for([&] {return rx.size() == 1;}));
  b.send("from b");
  ASSERT_TRUE(wait_for([&] {return rx.size() == 2;}));
  const Endpoint peer_a = *rx.chunks[0].second.endpoint;
  const Endpoint peer_b = *rx.chunks[1].second.endpoint;
  EXPECT_EQ(rx.chunks[0].first, "from a");
  EXPECT_NE(peer_a, peer_b);

  // A reply goes only to the client that asked; a frame without endpoint goes to everyone.
  EXPECT_TRUE(server.write(text_frame("to a", peer_a)));
  EXPECT_TRUE(server.write(text_frame("all")));
  EXPECT_EQ(a.receive(7), "to aall");
  EXPECT_EQ(b.receive(3), "all");

  // Disconnects are reported so the device can drop the client's partial frame.
  a.close();
  ASSERT_TRUE(wait_for([&] {return rx.size() == 3;}));
  EXPECT_TRUE(rx.chunks[2].second.attributes.contains(kDisconnectedAttribute));
  EXPECT_EQ(*rx.chunks[2].second.endpoint, peer_a);
  EXPECT_FALSE(server.write(text_frame("to a", peer_a)));
  server.close();
}

TEST(TcpServer, RefusesClientsBeyondTheLimit)
{
  Context ctx = make_context();
  TcpServer server;
  server.configure(Config::from_string("tcp: 127.0.0.1:0\nmax_clients: 1"), ctx);
  server.open();
  Client a(server.local_port());
  ASSERT_TRUE(wait_for([&] {return *server.stats().peer_connected;}));
  Client b(server.local_port());
  EXPECT_EQ(b.receive(1, 300ms), "");  // closed by the server
  EXPECT_TRUE(server.write(text_frame("x")));
  EXPECT_EQ(a.receive(1), "x");
}

std::vector<std::string> run_http(const std::vector<std::string> & chunks)
{
  Context ctx = make_context();
  Http framer;
  framer.configure(Config::from_string("max_size: 200"), ctx);
  std::vector<std::string> frames;
  for (const auto & c : chunks) {
    framer.feed(
      ByteView(reinterpret_cast<const std::uint8_t *>(c.data()), c.size()), {},
      [&](Frame && f) {frames.emplace_back(f.data.begin(), f.data.end());});
  }
  return frames;
}

TEST(HttpFramer, SplitsRequestsWithAndWithoutBodies)
{
  const std::string get = "GET /a HTTP/1.1\r\nHost: x\r\n\r\n";
  const std::string post = "POST /b HTTP/1.1\r\ncontent-LENGTH: 5\r\n\r\nhello";
  const auto frames = run_http({get.substr(0, 10), get.substr(10) + post.substr(0, 40),
        post.substr(40) + get});
  EXPECT_EQ(frames, (std::vector<std::string>{get, post, get}));
  // Oversized and chunked messages are dropped.
  EXPECT_TRUE(run_http({"POST /c HTTP/1.1\r\nContent-Length: 999\r\n\r\n"}).empty());
  EXPECT_EQ(run_http({"POST /c HTTP/1.1\r\nTransfer-Encoding: chunked\r\n\r\n" + get}),
    (std::vector<std::string>{get}));
}

TEST(HttpCodec, DecodesRequests)
{
  Context ctx = make_context();
  HttpCodec codec;
  codec.configure(Config::from_string("{}"), Config(), ctx);
  Record r = Record::make(codec.schema());
  const std::string req = "POST /api/v1/sensor/config?staging=true&reinit=false HTTP/1.1\r\n"
    "Host: 127.0.0.5\r\nContent-Length: 2\r\n\r\n{}";
  ASSERT_TRUE(codec.decode(text_frame(req), r).ok());
  EXPECT_EQ(std::get<std::string>(r.at("method")), "POST");
  EXPECT_EQ(std::get<std::string>(r.at("path")), "/api/v1/sensor/config");
  EXPECT_EQ(std::get<std::string>(r.at("query")), "staging=true&reinit=false");
  EXPECT_EQ(std::get<std::string>(r.at("body")), "{}");
  EXPECT_FALSE(codec.decode(text_frame("hello\r\n\r\n"), r).ok());
}

TEST(HttpCodec, EncodesResponsesFromConfigOrMapping)
{
  const auto dir = std::filesystem::temp_directory_path() / "simport_http_test";
  std::filesystem::create_directories(dir);
  std::ofstream(dir / "body.json") << R"({"fw": "v3.1.0"})";
  Context ctx = make_context();
  HttpCodec codec;
  codec.configure(
    Config::from_string("body_file: body.json\nheaders: {X-Test: yes}", "codec.yaml", dir),
        Config(),
    ctx);
  Record r = Record::make(codec.schema());
  std::vector<Frame> out;
  codec.encode(r, out);
  ASSERT_EQ(out.size(), 1u);
  EXPECT_EQ(std::string(out[0].data.begin(), out[0].data.end()),
    "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: 16\r\n"
    "X-Test: yes\r\n\r\n{\"fw\": \"v3.1.0\"}");

  r.at("status") = std::uint64_t{404};
  r.at("body") = std::string("nope");
  out.clear();
  codec.encode(r, out);
  EXPECT_EQ(std::string(out[0].data.begin(), out[0].data.end()),
    "HTTP/1.1 404 Not Found\r\nContent-Type: application/json\r\nContent-Length: 4\r\n"
    "X-Test: yes\r\n\r\nnope");

  HttpCodec bad;
  EXPECT_THROW(bad.configure(Config::from_string("status: 42"), Config(), ctx), ConfigError);
  EXPECT_THROW(
    bad.configure(Config::from_string("body_file: missing.json"), Config(), ctx), ConfigError);
  std::filesystem::remove_all(dir);
}

}  // namespace
}  // namespace simport::plugins
