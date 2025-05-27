#pragma once

#ifdef USE_ESP32

#include <freertos/FreeRTOS.h>
#include <freertos/event_groups.h>
#include <freertos/queue.h>

#include "esphome/components/audio/audio.h"
#include "esphome/components/speaker/speaker.h"

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"

#include "esphome/components/switch/switch.h"
#include <lwip/sockets.h>  

namespace esphome {
namespace udp_audio {

class UDPAudio : public speaker::Speaker, public Component {
 public:
  float get_setup_priority() const override { return esphome::setup_priority::PROCESSOR; }

  void setup() override;
  void loop() override;

  void set_ip_address(const std::string &ip) { this->ip_address_ = ip; }
  void set_port(uint16_t port) { this->port_ = port; }

  void set_tag_enable_switch(esphome::switch_::Switch *sw) { this->tag_enable_switch_ = sw; }
  bool get_tag_enable() const { 
    return this->tag_enable_switch_ != nullptr && this->tag_enable_switch_->state;
  }

  std::string get_ip_address() const { return this->ip_address_; }
  uint16_t get_port() const { return this->port_; }

  void start() override;
  void stop() override;
  void finish() override;

  void set_pause_state(bool pause_state) override { this->pause_state_ = pause_state; }
  bool get_pause_state() const override { return this->pause_state_; }

  /// @brief Plays the provided audio data.
  /// Starts the speaker task, if necessary. Transmits audio data via UDP.
  /// @param data Audio data in the format set by the parent speaker classes ``set_audio_stream_info`` method.
  /// @param length The length of the audio data in bytes.
  /// @param ticks_to_wait The FreeRTOS ticks to wait before trying to resend data if needed.
  /// @return The number of bytes that were actually sent.
  size_t play(const uint8_t *data, size_t length, TickType_t ticks_to_wait) override;
  size_t play(const uint8_t *data, size_t length) override { return play(data, length, 0); }

  bool has_buffered_data() const override;

  /// @brief Sets the volume of the speaker. 
  /// Note: Volume control is not supported in the UDP streaming implementation.
  /// @param volume between 0.0 and 1.0
  void set_volume(float volume) override;

  /// @brief Mutes or unmute the speaker.
  /// Note: Muting is not supported in the UDP streaming implementation.
  /// @param mute_state true for muting, false for unmuting
  void set_mute_state(bool mute_state) override;

 protected:
  /// @brief Function for the FreeRTOS task handling audio output.
  /// After receiving the COMMAND_START signal, it transitions to running state.
  /// Stops immediately after receiving the COMMAND_STOP signal and stops gracefully
  /// after receiving the COMMAND_STOP_GRACEFULLY signal. It communicates state
  /// and errors via event_group_.
  /// @param params UDPAudio component
  static void speaker_task(void *params);

  /// @brief Sends a stop command to the speaker task via event_group_.
  /// @param wait_on_empty If false, sends the COMMAND_STOP signal. If true, sends the COMMAND_STOP_GRACEFULLY signal.
  void stop_(bool wait_on_empty);

  /// @brief Sets the corresponding ERR_ESP event group bits.
  /// @param err esp_err_t error code.
  /// @return True if an ERR_ESP bit is set and false if err == ESP_OK
  bool send_esp_err_to_event_group_(esp_err_t err);

  /// @brief Deletes the speaker's task.
  /// Sets the STATE_STOPPED event bit and deletes the task.
  /// Should only be called by the speaker_task itself.
  /// @param buffer_size This parameter is kept for compatibility but not used in UDP implementation
  void delete_task_(size_t buffer_size);

  TaskHandle_t speaker_task_handle_{nullptr};
  EventGroupHandle_t event_group_{nullptr};
  bool task_created_{false};
  bool pause_state_{false};

  esphome::switch_::Switch *tag_enable_switch_{nullptr};
  int sock_{-1};
  struct sockaddr_in dest_addr_;
  bool udp_initialized_;
  std::string ip_address_;
  uint16_t port_;

  uint32_t accumulated_frames_written_{0};
};

}  // namespace udp_audio
}  // namespace esphome

#endif  // USE_ESP32