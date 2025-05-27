#include "udp_audio_speaker.h"

#ifdef USE_ESP32

#include "esphome/components/audio/audio.h"
#include "esphome/core/application.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <lwip/sockets.h>
#include <lwip/netdb.h>
#include "esphome/components/wifi/wifi_component.h"

namespace esphome {
namespace udp_audio {

static bool last_tag_enable_state = false; // Store last known state
static const char *const TAG = "udp_audio.speaker";

enum SpeakerEventGroupBits : uint32_t {
  COMMAND_START = (1 << 0),            // starts the speaker task
  COMMAND_STOP = (1 << 1),             // stops the speaker task
  COMMAND_STOP_GRACEFULLY = (1 << 2),  // Stops the speaker task once all data has been written
  STATE_STARTING = (1 << 10),
  STATE_RUNNING = (1 << 11),
  STATE_STOPPING = (1 << 12),
  STATE_STOPPED = (1 << 13),
  ERR_TASK_FAILED_TO_START = (1 << 14),
  ERR_ESP_INVALID_STATE = (1 << 15),
  ERR_ESP_NOT_SUPPORTED = (1 << 16),
  ERR_ESP_INVALID_ARG = (1 << 17),
  ERR_ESP_INVALID_SIZE = (1 << 18),
  ERR_ESP_NO_MEM = (1 << 19),
  ERR_ESP_FAIL = (1 << 20),
  ALL_ERR_ESP_BITS = ERR_ESP_INVALID_STATE | ERR_ESP_NOT_SUPPORTED | ERR_ESP_INVALID_ARG | ERR_ESP_INVALID_SIZE |
                     ERR_ESP_NO_MEM | ERR_ESP_FAIL,
  ALL_BITS = 0x00FFFFFF,  // All valid FreeRTOS event group bits
};

// Translates a SpeakerEventGroupBits ERR_ESP bit to the coressponding esp_err_t
static esp_err_t err_bit_to_esp_err(uint32_t bit) {
  switch (bit) {
    case SpeakerEventGroupBits::ERR_ESP_INVALID_STATE:
      return ESP_ERR_INVALID_STATE;
    case SpeakerEventGroupBits::ERR_ESP_INVALID_ARG:
      return ESP_ERR_INVALID_ARG;
    case SpeakerEventGroupBits::ERR_ESP_INVALID_SIZE:
      return ESP_ERR_INVALID_SIZE;
    case SpeakerEventGroupBits::ERR_ESP_NO_MEM:
      return ESP_ERR_NO_MEM;
    case SpeakerEventGroupBits::ERR_ESP_NOT_SUPPORTED:
      return ESP_ERR_NOT_SUPPORTED;
    default:
      return ESP_FAIL;
  }
}

void UDPAudio::setup() {
  ESP_LOGCONFIG(TAG, "Setting up I2S Audio Speaker...");
  ESP_LOGD(TAG, "Setting up I2S Audio Speaker...");

  this->event_group_ = xEventGroupCreate();

  // Set up UDP socket
  this->udp_initialized_ = false;  // Don't initialize UDP until WiFi is ready

  if (this->event_group_ == nullptr) {
    ESP_LOGE(TAG, "Failed to create event group");
    this->mark_failed();
    return;
  }
}

void UDPAudio::loop() {
  uint32_t event_group_bits = xEventGroupGetBits(this->event_group_);

  if (event_group_bits & SpeakerEventGroupBits::STATE_STARTING) {
    ESP_LOGD(TAG, "Starting Speaker");
    this->state_ = speaker::STATE_STARTING;
    xEventGroupClearBits(this->event_group_, SpeakerEventGroupBits::STATE_STARTING);
  }
  if (event_group_bits & SpeakerEventGroupBits::STATE_RUNNING) {
    ESP_LOGD(TAG, "Started Speaker");
    this->state_ = speaker::STATE_RUNNING;
    xEventGroupClearBits(this->event_group_, SpeakerEventGroupBits::STATE_RUNNING);
    this->status_clear_warning();
    this->status_clear_error();
  }
  if (event_group_bits & SpeakerEventGroupBits::STATE_STOPPING) {
    ESP_LOGD(TAG, "Stopping Speaker");
    this->state_ = speaker::STATE_STOPPING;
    xEventGroupClearBits(this->event_group_, SpeakerEventGroupBits::STATE_STOPPING);
  }
  if (event_group_bits & SpeakerEventGroupBits::STATE_STOPPED) {
    if (!this->task_created_) {
      ESP_LOGD(TAG, "Stopped Speaker");
      this->state_ = speaker::STATE_STOPPED;
      xEventGroupClearBits(this->event_group_, SpeakerEventGroupBits::ALL_BITS);
      this->speaker_task_handle_ = nullptr;
    }
  }

  if (event_group_bits & SpeakerEventGroupBits::ERR_TASK_FAILED_TO_START) {
    this->status_set_error("Failed to start speaker task");
    xEventGroupClearBits(this->event_group_, SpeakerEventGroupBits::ERR_TASK_FAILED_TO_START);
  }

  if (event_group_bits & SpeakerEventGroupBits::ALL_ERR_ESP_BITS) {
    uint32_t error_bits = event_group_bits & SpeakerEventGroupBits::ALL_ERR_ESP_BITS;
    ESP_LOGW(TAG, "Error: %s", esp_err_to_name(err_bit_to_esp_err(error_bits)));
    this->status_set_warning();
  }

  xEventGroupClearBits(this->event_group_, ALL_ERR_ESP_BITS);

  // Configure UDP socket when WiFi is connected
  if (!this->udp_initialized_ && esphome::wifi::global_wifi_component->is_connected()) {
    ESP_LOGD(TAG, "WiFi connected. Initializing UDP socket...");
    
    this->sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (this->sock_ < 0) {
        ESP_LOGE(TAG, "Failed to create socket");
        return;
    }

    this->dest_addr_.sin_addr.s_addr = inet_addr(this->get_ip_address().c_str());
    this->dest_addr_.sin_family = AF_INET;
    this->dest_addr_.sin_port = htons(this->get_port());

    ESP_LOGD(TAG, "UDP Socket created for %s:%d", this->get_ip_address().c_str(), this->get_port());
    this->udp_initialized_ = true;
  }
}

void UDPAudio::set_volume(float volume) {
  // Store the volume value but never apply it
  this->volume_ = volume;
  ESP_LOGD(TAG, "Volume adjustment not supported for UDP streaming");
}

void UDPAudio::set_mute_state(bool mute_state) {
  // Store the mute state but never apply it
  this->mute_state_ = mute_state;
  ESP_LOGD(TAG, "Mute not supported for UDP streaming");
}

size_t UDPAudio::play(const uint8_t *data, size_t length, TickType_t ticks_to_wait) {
  static uint32_t last_log_time = 0;  // Stores the last log timestamp
  
  if (this->is_failed()) {
    uint32_t now = millis();
    if (now - last_log_time >= 3000) {  // Check if 3 seconds have passed
      ESP_LOGE(TAG, "Cannot play audio, speaker failed to setup");
      last_log_time = now;  // Update the last log time
    }
    return 0;
  }

  if (this->state_ != speaker::STATE_RUNNING && this->state_ != speaker::STATE_STARTING) {
    this->start();
  }

  if (this->state_ != speaker::STATE_RUNNING) {
    uint32_t now = millis();
    if (now - last_log_time >= 3000) {
      ESP_LOGD(TAG, "Play Delay... state = %d", this->state_);
      last_log_time = now;
    }
    vTaskDelay(ticks_to_wait);
    ticks_to_wait = 0;
    return 0;
  }

  size_t bytes_written = 0;
  if (this->state_ == speaker::STATE_RUNNING) {
    bool current_tag_enable_state = this->get_tag_enable(); // Get the latest state

    if (current_tag_enable_state != last_tag_enable_state) {  // Log only when state changes
      ESP_LOGD(TAG, "Tag Enable changed: %s", current_tag_enable_state ? "ON" : "OFF");
      last_tag_enable_state = current_tag_enable_state;  // Update last known state
    }

    if (current_tag_enable_state) {
      // Always send audio data regardless of mute state
      int err = sendto(this->sock_, data, length, 0, 
                     (struct sockaddr *)&this->dest_addr_, sizeof(this->dest_addr_));
      if (err < 0) {
        ESP_LOGE(TAG, "UDP send failed: errno %d", errno);
      } else {
        bytes_written = length;
      }
    }
    
    // Calculate playback timing for callbacks even if we didn't send data
    this->accumulated_frames_written_ += this->audio_stream_info_.bytes_to_frames(bytes_written);
    const uint32_t new_playback_ms =
        this->audio_stream_info_.frames_to_milliseconds_with_remainder(&this->accumulated_frames_written_);
    const uint32_t remainder_us =
        this->audio_stream_info_.frames_to_microseconds(this->accumulated_frames_written_);

    uint32_t pending_frames = 0; // No buffer, so no pending frames
    const uint32_t pending_ms = 0;
    uint32_t write_timestamp = micros();

    this->audio_output_callback_(new_playback_ms, remainder_us, pending_ms, write_timestamp);
  }

  return bytes_written;
}

bool UDPAudio::has_buffered_data() const {
  // No buffering in UDP-only version
  return false;
}

void UDPAudio::speaker_task(void *params) {
  UDPAudio *this_speaker = (UDPAudio *) params;
  this_speaker->task_created_ = true;

  uint32_t event_group_bits =
      xEventGroupWaitBits(this_speaker->event_group_,
                          SpeakerEventGroupBits::COMMAND_START | SpeakerEventGroupBits::COMMAND_STOP |
                              SpeakerEventGroupBits::COMMAND_STOP_GRACEFULLY,
                          pdTRUE,
                          pdFALSE,
                          portMAX_DELAY);

  if (event_group_bits & (SpeakerEventGroupBits::COMMAND_STOP | SpeakerEventGroupBits::COMMAND_STOP_GRACEFULLY)) {
    // Received a stop signal before the task was requested to start
    this_speaker->delete_task_(0);
  }

  xEventGroupSetBits(this_speaker->event_group_, SpeakerEventGroupBits::STATE_STARTING);
  
  // Since we're not using I2S, we can immediately transition to running state
  xEventGroupSetBits(this_speaker->event_group_, SpeakerEventGroupBits::STATE_RUNNING);

  // This is a minimal task that just waits for stop commands
  while (true) {
    event_group_bits = xEventGroupGetBits(this_speaker->event_group_);
    
    if (event_group_bits & (SpeakerEventGroupBits::COMMAND_STOP | SpeakerEventGroupBits::COMMAND_STOP_GRACEFULLY)) {
      xEventGroupClearBits(this_speaker->event_group_, 
                          SpeakerEventGroupBits::COMMAND_STOP | SpeakerEventGroupBits::COMMAND_STOP_GRACEFULLY);
      break;
    }
    
    // Check if audio_stream_info has changed
    // No I2S reconfiguration needed in this implementation
    
    // Sleep to yield CPU time
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  xEventGroupSetBits(this_speaker->event_group_, SpeakerEventGroupBits::STATE_STOPPING);
  
  this_speaker->delete_task_(0);
}

void UDPAudio::start() {
  ESP_LOGD(TAG, "Starting Speaker");
  if (!this->is_ready() || this->is_failed() || this->status_has_error())
    return;
  if ((this->state_ == speaker::STATE_STARTING) || (this->state_ == speaker::STATE_RUNNING))
    return;

  static const size_t TASK_STACK_SIZE = 4096;
  static const ssize_t TASK_PRIORITY = 23;

  if (!this->task_created_ && (this->speaker_task_handle_ == nullptr)) {
    xTaskCreate(UDPAudio::speaker_task, "speaker_task", TASK_STACK_SIZE, (void *) this, TASK_PRIORITY,
                &this->speaker_task_handle_);

    if (this->speaker_task_handle_ != nullptr) {
      xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::COMMAND_START);
    } else {
      xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::ERR_TASK_FAILED_TO_START);
    }
  }
}

void UDPAudio::stop() { 
  ESP_LOGD(TAG, "Stop");
  this->stop_(false); 
}

void UDPAudio::finish() { 
  ESP_LOGD(TAG, "Finish...");
  this->stop_(true); 
}

void UDPAudio::stop_(bool wait_on_empty) {
  ESP_LOGD(TAG, "stop_");
  if (this->is_failed())
    return;
  if (this->state_ == speaker::STATE_STOPPED)
    return;

  if (wait_on_empty) {
    xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::COMMAND_STOP_GRACEFULLY);
  } else {
    xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::COMMAND_STOP);
  }
}

bool UDPAudio::send_esp_err_to_event_group_(esp_err_t err) {
  switch (err) {
    case ESP_OK:
      return false;
    case ESP_ERR_INVALID_STATE:
      xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::ERR_ESP_INVALID_STATE);
      return true;
    case ESP_ERR_INVALID_ARG:
      xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::ERR_ESP_INVALID_ARG);
      return true;
    case ESP_ERR_INVALID_SIZE:
      xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::ERR_ESP_INVALID_SIZE);
      return true;
    case ESP_ERR_NO_MEM:
      xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::ERR_ESP_NO_MEM);
      return true;
    case ESP_ERR_NOT_SUPPORTED:
      xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::ERR_ESP_NOT_SUPPORTED);
      return true;
    default:
      xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::ERR_ESP_FAIL);
      return true;
  }
}

void UDPAudio::delete_task_(size_t buffer_size) {
  ESP_LOGD(TAG, "Delete Task...");

  xEventGroupSetBits(this->event_group_, SpeakerEventGroupBits::STATE_STOPPED);

  this->task_created_ = false;
  vTaskDelete(nullptr);
}

}  // namespace udp_audio
}  // namespace esphome

#endif  // USE_ESP32