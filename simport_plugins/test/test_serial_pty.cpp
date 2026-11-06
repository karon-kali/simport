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

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <optional>
#include <string_view>
#include <vector>

#include "simport_plugins/serial_pty.hpp"

using namespace std::chrono_literals;
namespace fs = std::filesystem;

namespace simport::plugins
{
namespace
{

/// Wait up to `timeout` for `pred` to become true.
template<typename Pred>
bool wait_for(Pred pred, std::chrono::milliseconds timeout = 2000ms)
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

/// Read whatever arrives on `fd` within `timeout`.
Bytes read_some(int fd, std::chrono::milliseconds timeout = 500ms)
{
  Bytes out;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    pollfd pfd{fd, POLLIN, 0};
    if (::poll(&pfd, 1, 20) > 0 && (pfd.revents & POLLIN)) {
      std::uint8_t buf[256];
      const ssize_t n = ::read(fd, buf, sizeof(buf));
      if (n > 0) {
        out.insert(out.end(), buf, buf + n);
      }
    } else if (!out.empty()) {
      break;
    }
  }
  return out;
}

class SerialPtyTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    dir_ = fs::path(::testing::TempDir()) / ("simport_pty_" + std::to_string(::getpid()));
    link_ = dir_ / "dev0";
    Context ctx("dev", std::make_shared<NullLogger>(), std::make_shared<SystemClock>());
    pty_.configure(
      Config::from_string("plugin: x\nserial: " + link_.string() + "\nbaud: 9600\nframing: 7E2\n"),
      ctx);
  }

  void TearDown() override
  {
    pty_.close();
    fs::remove_all(dir_);
  }

  int open_driver_side()
  {
    const int fd = ::open(link_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    EXPECT_GE(fd, 0) << "cannot open " << link_;
    return fd;
  }

  fs::path dir_;
  fs::path link_;
  SerialPty pty_;
};

TEST_F(SerialPtyTest, CapsFromConfig)
{
  const TransportCaps caps = pty_.caps();
  EXPECT_TRUE(caps.stream);
  EXPECT_EQ(caps.line_rate_bps, 9600.0);
  EXPECT_DOUBLE_EQ(caps.bits_per_byte, 11.0);  // start + 7 data + parity + 2 stop
}

TEST_F(SerialPtyTest, CreatesAndRemovesLink)
{
  pty_.open();
  ASSERT_TRUE(fs::is_symlink(link_));
  EXPECT_EQ(fs::read_symlink(link_).string(), pty_.slave_name());
  EXPECT_NE(pty_.describe().find(link_.string()), std::string::npos);
  pty_.close();
  EXPECT_FALSE(fs::exists(fs::symlink_status(link_)));
  EXPECT_FALSE(pty_.is_open());
}

TEST_F(SerialPtyTest, ReplacesStaleLinkButNotRegularFiles)
{
  fs::create_directories(dir_);
  fs::create_symlink("/dev/null-gone", link_);
  pty_.open();
  EXPECT_EQ(fs::read_symlink(link_).string(), pty_.slave_name());
  pty_.close();

  {std::FILE * f = std::fopen(link_.c_str(), "w"); std::fclose(f);}
  EXPECT_THROW(pty_.open(), TransportError);
  EXPECT_TRUE(fs::is_regular_file(link_));
}

TEST_F(SerialPtyTest, DropsWhileNoDriverAndNeverDeliversStaleData)
{
  pty_.open();
  EXPECT_FALSE(pty_.write(Frame{{1, 2, 3}, {}, {}}));
  EXPECT_EQ(pty_.stats().tx_dropped, 1u);

  const int fd = open_driver_side();
  ASSERT_TRUE(wait_for([&] {return pty_.peer_connected();}));
  ASSERT_TRUE(pty_.write(Frame{{0xAA, 0x55}, {}, {}}));
  EXPECT_EQ(read_some(fd), (Bytes{0xAA, 0x55}));  // no 1, 2, 3 before it
  EXPECT_EQ(pty_.stats().frames_tx, 1u);
  EXPECT_EQ(pty_.stats().bytes_tx, 2u);

  ::close(fd);
  ASSERT_TRUE(wait_for([&] {return !pty_.peer_connected();}));
  EXPECT_FALSE(pty_.write(Frame{{9}, {}, {}}));
}

TEST_F(SerialPtyTest, BinarySafeRawMode)
{
  pty_.open();
  const int fd = open_driver_side();
  ASSERT_TRUE(wait_for([&] {return pty_.peer_connected();}));
  // Bytes a cooked tty would translate or interpret: CR, LF, ^C, ^D, DEL, NUL.
  const Bytes tricky{0x0D, 0x0A, 0x03, 0x04, 0x7F, 0x00, 0xFF};
  ASSERT_TRUE(pty_.write(Frame{tricky, {}, {}}));
  EXPECT_EQ(read_some(fd), tricky);
  ::close(fd);
}

TEST_F(SerialPtyTest, ReceivesDriverWrites)
{
  std::mutex m;
  Bytes received;
  pty_.set_receive_callback(
    [&](ByteView data, const FrameMeta &, Timestamp stamp) {
      EXPECT_GT(stamp.count(), 0);
      std::lock_guard<std::mutex> lock(m);
      received.insert(received.end(), data.begin(), data.end());
    });
  pty_.open();
  const int fd = open_driver_side();
  const std::uint8_t cmd[] = {0x10, 0x20, 0x30};
  ASSERT_EQ(::write(fd, cmd, sizeof(cmd)), 3);
  ASSERT_TRUE(wait_for([&] {std::lock_guard<std::mutex> lock(m); return received.size() == 3;}));
  EXPECT_EQ(received, (Bytes{0x10, 0x20, 0x30}));
  EXPECT_EQ(pty_.stats().bytes_rx, 3u);
  ::close(fd);
}

class CaptureLogger : public Logger
{
public:
  void log(LogLevel, std::string_view m) override
  {
    std::lock_guard<std::mutex> lock(mutex);
    lines.emplace_back(m);
  }
  bool enabled(LogLevel) const override {return true;}
  bool contains(const std::string & needle)
  {
    std::lock_guard<std::mutex> lock(mutex);
    for (const auto & l : lines) {
      if (l.find(needle) != std::string::npos) {return true;}
    }
    return false;
  }
  std::mutex mutex;
  std::vector<std::string> lines;
};

TEST_F(SerialPtyTest, WarnsWhenDriverUsesWrongBaud)
{
  auto logger = std::make_shared<CaptureLogger>();
  Context ctx("dev", logger, std::make_shared<SystemClock>());
  SerialPty pty;
  pty.configure(Config::from_string("serial: " + link_.string() + "\nbaud: 115200\n"), ctx);
  pty.open();
  // A driver that keeps the defaults it finds (raw, 115200 8N1): no warning.
  int fd = open_driver_side();
  std::this_thread::sleep_for(800ms);
  EXPECT_FALSE(logger->contains("baud"));
  ::close(fd);
  ASSERT_TRUE(wait_for([&] {return !pty.peer_connected();}));
  // A driver that configures 9600 baud.
  fd = open_driver_side();
  termios tio{};
  ::tcgetattr(fd, &tio);
  ::cfsetspeed(&tio, B9600);
  ::tcsetattr(fd, TCSANOW, &tio);
  EXPECT_TRUE(wait_for([&] {return logger->contains("for 9600 baud 8N1");}));
  EXPECT_TRUE(logger->contains("device is 115200 baud 8N1"));
  EXPECT_EQ(pty.stats().peer_connected, std::optional<bool>(true));
  ::close(fd);
  pty.close();
  EXPECT_FALSE(pty.stats().peer_connected.has_value());
}

TEST(SerialFraming, Parse)
{
  EXPECT_DOUBLE_EQ(SerialFraming::parse("8N1").bits_per_byte(), 10.0);
  EXPECT_THROW(SerialFraming::parse("9N1"), std::invalid_argument);
  EXPECT_THROW(SerialFraming::parse("8X1"), std::invalid_argument);
  EXPECT_THROW(SerialFraming::parse("8N"), std::invalid_argument);
}

TEST(SerialPtyConfig, Errors)
{
  Context ctx("dev", std::make_shared<NullLogger>(), std::make_shared<SystemClock>());
  SerialPty pty;
  EXPECT_THROW(pty.configure(Config::from_string("plugin: x\n"), ctx), ConfigError);
  EXPECT_THROW(
    pty.configure(Config::from_string("serial: /tmp/x\nframing: 8Q1\n"), ctx), ConfigError);
  EXPECT_THROW(pty.configure(Config::from_string("serial: /tmp/x\nbuad: 9600\n"), ctx),
    ConfigError);
}

}  // namespace
}  // namespace simport::plugins
