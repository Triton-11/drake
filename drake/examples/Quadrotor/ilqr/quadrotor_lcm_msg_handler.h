#pragma once

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "drake/lcm/drake_lcm.h"
#include "drake/lcm/drake_lcm_message_handler_interface.h"
#include "drake/lcmt_quadrotor_state.hpp"
#include "drake/lcmt_quadrotor_ab_matrices.hpp"

namespace drake {
namespace examples {
namespace lqr {

/// Handles received LCM messages of type lcmt_quadrotor_state.
// copied from drake/lcm/test/drake_lcm_test.cc
class MessageHandler : public lcm::DrakeLcmMessageHandlerInterface {
 public:
  /// A constructor that initializes the memory for storing received LCM
  /// messages.
  MessageHandler() {
    // Initializes the fields of received_message.
    received_message_.n_inputs = 0;
    received_message_.n_states = 0;
    received_message_.input_vector.resize(received_message_.n_inputs);
    received_message_.state_vector.resize(received_message_.n_states);
    received_message_.timestamp = 0;
  }

  /// This is the callback method.
  void HandleMessage(const std::string& channel, const void* message_buffer,
                     int message_size) override {
    channel_ = channel;
    std::lock_guard<std::mutex> lock(message_mutex_);
    received_message_.decode(message_buffer, 0, message_size);
    //std::cout << "HandleMessage count b " << received_message_count_ << std::endl;
    received_message_count_++;
    //std::cout << "HandleMessage count a " << received_message_count_ << std::endl;

    received_message_condition_variable_.notify_all();
    std::cout << "HandleMessage received message" << std::endl;
  }

  /// Returns a copy of the most recently received message.
  lcmt_quadrotor_state GetReceivedMessage() {
    lcmt_quadrotor_state message_copy;
    std::lock_guard<std::mutex> lock(message_mutex_);
    message_copy = received_message_;
    return message_copy;
  }

  /// Returns the channel on which the most recent message was received.
  const std::string& get_receive_channel() { return channel_; }

  int WaitForMessage(int old_message_count) const {
    // The message buffer and counter are updated in HandleMessage(), which is
    // a callback function invoked by a different thread owned by the
    // drake::lcm::DrakeLcmInterface instance passed to the constructor. Thus,
    // for thread safety, these need to be properly protected by a mutex.
    std::unique_lock<std::mutex> lock(message_mutex_);

    // This while loop is necessary to guard for spurious wakeup:
    // https://en.wikipedia.org/wiki/Spurious_wakeup
    //std::cout << "old_message_count " << old_message_count << std::endl;
    //std::cout << "received_message_count_ " << received_message_count_ << std::endl;
    while (old_message_count == received_message_count_)
      // When wait returns, lock is atomically acquired. So it's thread safe to
      // read received_message_count_.
      received_message_condition_variable_.wait(lock);
    int new_message_count = received_message_count_;
    lock.unlock();

    return new_message_count;
  }

 private:
  std::string channel_{};
  mutable std::mutex message_mutex_;
  lcmt_quadrotor_state received_message_;
  int received_message_count_{0};
  mutable std::condition_variable received_message_condition_variable_;

};

}  // namespace acrobot
}  // namespace examples
}  // namespace drake
