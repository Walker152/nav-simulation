// Copyright 2019 Open Source Robotics Foundation, Inc.
// Copyright 2026 Naturewill contributors
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

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <future>
#include <memory>
#include <pthread.h>
#include <signal.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/component_manager.hpp"

int main(int argc, char ** argv)
{
  // All ROS/DDS threads inherit this mask. Consume signals on the main thread
  // instead of letting rclcpp tear down entities while executor workers wait.
  sigset_t signals;
  sigemptyset(&signals);
  sigaddset(&signals, SIGINT);
  sigaddset(&signals, SIGTERM);
  const int mask_error = pthread_sigmask(SIG_BLOCK, &signals, nullptr);
  if (mask_error != 0) {
    std::fprintf(stderr, "Cannot block shutdown signals: %s\n", std::strerror(mask_error));
    return 1;
  }

  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
  auto manager = std::make_shared<rclcpp_components::ComponentManager>();
  const auto thread_num = manager->has_parameter("thread_num") ?
    manager->get_parameter("thread_num").as_int() : 0;
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
    rclcpp::ExecutorOptions{}, thread_num);
  manager->set_executor(executor);
  executor->add_node(manager);
  auto spinning = std::async(std::launch::async, [&executor]() {executor->spin();});
  int result = 0;
  const timespec signal_wait{0, 50000000};
  while (spinning.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
    const int received = sigtimedwait(&signals, nullptr, &signal_wait);
    if (received == SIGINT || received == SIGTERM) {
      break;
    }
    if (received == -1 && errno != EAGAIN && errno != EINTR) {
      std::fprintf(stderr, "Cannot wait for shutdown signals: %s\n", std::strerror(errno));
      result = 1;
      break;
    }
  }
  // cancel() can precede spin() entering its loop. Keep cancelling until the
  // entire spin task (including all MT workers) has returned.
  try {
    while (spinning.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
      executor->cancel();
      spinning.wait_for(std::chrono::milliseconds(50));
    }
  } catch (const std::exception & error) {
    std::fprintf(stderr, "Cannot cancel simulation executor: %s\n", error.what());
    result = 1;
  }
  try {
    spinning.get();
  } catch (const std::exception & error) {
    std::fprintf(stderr, "Simulation executor failed: %s\n", error.what());
    result = 1;
  }
  // Drop the wait set and weak callback-group associations while components
  // still exist. Nav2 cleanup then destroys groups without a stale executor map.
  executor.reset();
  rclcpp::shutdown();
  manager.reset();
  return result;
}
