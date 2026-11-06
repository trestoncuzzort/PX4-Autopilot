/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "log_writer_file.h"

#include <gtest/gtest.h>
#include <algorithm>
#include <chrono>
#include <cerrno>
#include <fcntl.h>
#include <memory>
#include <poll.h>
#include <string>
#include <sys/stat.h>
#include <thread>
#include <unistd.h>
#include <vector>

using px4::logger::LogType;
using px4::logger::LogWriterFile;
using namespace std::chrono_literals;

class LogWriterFileTest : public ::testing::Test
{
protected:
	void SetUp() override
	{
		char directory[] = "/tmp/px4-logger-test-XXXXXX";
		ASSERT_NE(mkdtemp(directory), nullptr);
		_directory = directory;
		_fifo = _directory + "/log";
		ASSERT_EQ(mkfifo(_fifo.c_str(), 0600), 0);
		_reader = open(_fifo.c_str(), O_RDONLY | O_NONBLOCK);
		ASSERT_GE(_reader, 0);
		_capacity = fcntl(_reader, F_GETPIPE_SZ);
		ASSERT_GT(_capacity, 1);
		_writer.reset(new LogWriterFile(3 * _capacity));
		ASSERT_TRUE(_writer->init());
		ASSERT_TRUE(_writer->start_log(LogType::Full, _fifo.c_str()));
		ASSERT_EQ(_writer->thread_start(), 0);
		_running = true;
	}

	void TearDown() override
	{
		if (_running) {
			stopAndDrain();
		}

		_writer.reset();

		if (_reader >= 0) {
			close(_reader);
		}

		if (!_fifo.empty()) {
			unlink(_fifo.c_str());
		}

		if (!_directory.empty()) {
			rmdir(_directory.c_str());
		}
	}

	void append(std::vector<uint8_t> &bytes)
	{
		_writer->lock();
		const int result = _writer->write_message(LogType::Full, bytes.data(), bytes.size());
		_writer->unlock();
		ASSERT_EQ(result, 0);
		_writer->notify();
	}

	bool waitForWrite()
	{
		const auto deadline = std::chrono::steady_clock::now() + 5s;

		while (std::chrono::steady_clock::now() < deadline) {
			_writer->notify();
			pollfd descriptor{_reader, POLLIN, 0};

			if (poll(&descriptor, 1, 50) > 0) {
				uint8_t byte;

				if (read(_reader, &byte, 1) == 1) {
					_received.push_back(byte);
					return true;
				}
			}
		}

		return false;
	}

	void stopAndDrain()
	{
		std::thread reader([this]() {
			const auto deadline = std::chrono::steady_clock::now() + 10s;

			while (std::chrono::steady_clock::now() < deadline) {
				pollfd descriptor{_reader, POLLIN, 0};

				if (poll(&descriptor, 1, 100) <= 0) {
					continue;
				}

				uint8_t bytes[4096];
				const ssize_t count = read(_reader, bytes, sizeof(bytes));

				if (count > 0) {
					_received.insert(_received.end(), bytes, bytes + count);

				} else if (count == 0) {
					_saw_eof = true;
					break;

				} else if (errno != EAGAIN && errno != EINTR) {
					break;
				}
			}
		});
		_writer->thread_stop();
		reader.join();
		_running = false;
	}

	std::unique_ptr<LogWriterFile> _writer;
	std::vector<uint8_t> _received;
	int _capacity{0};
	bool _saw_eof{false};

private:
	std::string _directory;
	std::string _fifo;
	int _reader{-1};
	bool _running{false};
};

TEST_F(LogWriterFileTest, StopFlushesDataAppendedDuringWrite)
{
	std::vector<uint8_t> first(2 * _capacity, 0x1b);
	std::vector<uint8_t> tail(_capacity / 2, 0xab);
	append(first);
	ASSERT_TRUE(waitForWrite());

	// Seeing the first byte proves write() has started. Twice the pipe capacity cannot finish
	// until we drain it, so the appended tail is absent from that write's captured byte count.
	append(tail);
	_writer->stop_log(LogType::Full);
	stopAndDrain();

	EXPECT_TRUE(_saw_eof);
	ASSERT_EQ(_received.size(), first.size() + tail.size());
	EXPECT_TRUE(std::equal(first.begin(), first.end(), _received.begin()));
	EXPECT_TRUE(std::equal(tail.begin(), tail.end(), _received.begin() + first.size()));
}
