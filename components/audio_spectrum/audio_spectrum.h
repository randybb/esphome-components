#pragma once

#ifdef USE_ESP32

#include "esphome/components/speaker/speaker.h"
#include "esphome/core/component.h"

#include <complex>
#include <cstdint>
#include <vector>

namespace esphome::audio_spectrum {

/// Pass-through speaker that forwards audio unchanged to `output_speaker` and computes a spectrum of it.
///
/// The FFT runs in the task that feeds the speaker (e.g. the mixer task), not in the main loop.
/// `get_level(band)` returns 0..1 per band for a spectrum analyzer display.
class AudioSpectrum : public Component, public speaker::Speaker {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_output_speaker(speaker::Speaker *output) { this->output_ = output; }
  void set_bands(uint8_t bands) { this->bands_ = bands; }
  void set_frequency_range(float min_hz, float max_hz) {
    this->min_hz_ = min_hz;
    this->max_hz_ = max_hz;
  }

  size_t play(const uint8_t *data, size_t length, TickType_t ticks_to_wait) override;
  size_t play(const uint8_t *data, size_t length) override { return this->play(data, length, 0); }
  void start() override;
  void stop() override;
  void finish() override;
  bool has_buffered_data() const override { return this->output_->has_buffered_data(); }
  void set_pause_state(bool pause_state) override { this->output_->set_pause_state(pause_state); }
  bool get_pause_state() const override { return this->output_->get_pause_state(); }
  void set_volume(float volume) override;
  float get_volume() override { return this->output_->get_volume(); }
  void set_mute_state(bool mute_state) override;
  bool get_mute_state() override { return this->output_->get_mute_state(); }

  uint8_t get_bands() const { return this->bands_; }
  /// Level of a band, 0..1. Returns 0 when no audio was analysed recently.
  float get_level(uint8_t band) const;

 protected:
  void analyse_(const uint8_t *data, size_t length);
  void compute_spectrum_();

  speaker::Speaker *output_{nullptr};
  uint8_t bands_{20};
  float min_hz_{60.0f};
  float max_hz_{16000.0f};

  std::vector<float> samples_;  // mono samples waiting for the next FFT
  std::vector<std::complex<float>> fft_;
  std::vector<float> window_;
  std::vector<uint16_t> band_edges_;  // FFT bin index where each band starts, bands_ + 1 entries
  std::vector<float> levels_;
  uint32_t sample_rate_{0};  // effective rate after decimation
  uint8_t decimation_{1};
  float half_{0.0f};
  bool have_half_{false};
  volatile uint32_t last_analysis_ms_{0};
};

}  // namespace esphome::audio_spectrum

#endif
