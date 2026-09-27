#include "audio_spectrum.h"

#ifdef USE_ESP32

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

#include <algorithm>
#include <cmath>

namespace esphome::audio_spectrum {

static const char *const TAG = "audio_spectrum";

static constexpr size_t FFT_SIZE = 512;  // after 2x decimation of 48 kHz audio: 47 Hz bins, ~23 FFTs/s
static constexpr float FLOOR_DB = -60.0f;       // level 0
static constexpr float TILT_DB_PER_OCTAVE = 3.0f;  // music has less energy up high, lift it for a flatter look
static constexpr float RELEASE = 0.88f;         // per-FFT fall-off of the bars
static constexpr uint32_t STALE_MS = 250;

// In-place iterative radix-2 FFT, n must be a power of two
static void fft(std::complex<float> *a, size_t n) {
  for (size_t i = 1, j = 0; i < n; i++) {
    size_t bit = n >> 1;
    for (; j & bit; bit >>= 1)
      j ^= bit;
    j ^= bit;
    if (i < j)
      std::swap(a[i], a[j]);
  }
  for (size_t len = 2; len <= n; len <<= 1) {
    float ang = -2.0f * float(M_PI) / float(len);
    std::complex<float> wlen(std::cos(ang), std::sin(ang));
    for (size_t i = 0; i < n; i += len) {
      std::complex<float> w(1.0f, 0.0f);
      for (size_t j = 0; j < len / 2; j++) {
        std::complex<float> u = a[i + j], v = a[i + j + len / 2] * w;
        a[i + j] = u + v;
        a[i + j + len / 2] = u - v;
        w *= wlen;
      }
    }
  }
}

void AudioSpectrum::setup() {
  this->samples_.reserve(FFT_SIZE);
  this->fft_.resize(FFT_SIZE);
  this->window_.resize(FFT_SIZE);
  for (size_t i = 0; i < FFT_SIZE; i++)
    this->window_[i] = 0.5f - 0.5f * std::cos(2.0f * float(M_PI) * float(i) / float(FFT_SIZE - 1));
  this->levels_.assign(this->bands_, 0.0f);
  this->band_edges_.assign(this->bands_ + 1, 0);

  // timing callbacks of the real output (used by the media player) must reach whoever feeds this speaker
  this->output_->add_audio_output_callback(
      [this](uint32_t frames, int64_t timestamp) { this->audio_output_callback_.call(frames, timestamp); });
}

void AudioSpectrum::loop() {
  // mirror the output speaker state; STARTING / STOPPING are set by start() / stop()
  if (this->output_->is_running()) {
    this->state_ = speaker::STATE_RUNNING;
  } else if (this->output_->is_stopped() && this->state_ != speaker::STATE_STARTING) {
    this->state_ = speaker::STATE_STOPPED;
  }
}

void AudioSpectrum::dump_config() {
  ESP_LOGCONFIG(TAG,
                "Audio Spectrum:\n"
                "  Bands: %u\n"
                "  Range: %.0f - %.0f Hz\n"
                "  FFT size: %u",
                this->bands_, this->min_hz_, this->max_hz_, (unsigned) FFT_SIZE);
}

void AudioSpectrum::start() {
  this->output_->set_audio_stream_info(this->audio_stream_info_);
  this->output_->start();
  this->state_ = speaker::STATE_STARTING;
}

void AudioSpectrum::stop() {
  this->output_->stop();
  this->state_ = speaker::STATE_STOPPING;
}

void AudioSpectrum::finish() {
  this->output_->finish();
  this->state_ = speaker::STATE_STOPPING;
}

void AudioSpectrum::set_volume(float volume) {
  this->volume_ = volume;
  this->output_->set_volume(volume);
}

void AudioSpectrum::set_mute_state(bool mute_state) {
  this->mute_state_ = mute_state;
  this->output_->set_mute_state(mute_state);
}

size_t AudioSpectrum::play(const uint8_t *data, size_t length, TickType_t ticks_to_wait) {
  // Speakers like i2s_audio start themselves on play() without start() being called, using whatever stream info
  // they hold. set_audio_stream_info() is not virtual, so it only reached this object; pass it on.
  if (this->output_->get_audio_stream_info() != this->audio_stream_info_)
    this->output_->set_audio_stream_info(this->audio_stream_info_);
  size_t written = this->output_->play(data, length, ticks_to_wait);
  if (written > 0)
    this->analyse_(data, written);
  return written;
}

void AudioSpectrum::analyse_(const uint8_t *data, size_t length) {
  const auto &info = this->audio_stream_info_;
  const size_t bytes_per_sample = info.get_bits_per_sample() / 8;
  const size_t channels = info.get_channels();
  if (bytes_per_sample == 0 || channels == 0)
    return;
  const size_t frame_bytes = bytes_per_sample * channels;
  this->decimation_ = info.get_sample_rate() >= 32000 ? 2 : 1;

  for (size_t off = 0; off + frame_bytes <= length; off += frame_bytes) {
    // average the channels, using the most significant 16 bits of each little-endian sample
    int32_t sum = 0;
    for (size_t ch = 0; ch < channels; ch++) {
      const uint8_t *s = data + off + ch * bytes_per_sample + bytes_per_sample - 2;
      sum += int16_t(uint16_t(s[0]) | (uint16_t(s[1]) << 8));
    }
    // decimate high sample rates by 2 to keep the FFT cheap; the display tops out at max_frequency anyway
    const float sample = float(sum) / float(channels) / 32768.0f;
    if (this->decimation_ == 2) {
      if (!this->have_half_) {
        this->half_ = sample;
        this->have_half_ = true;
        continue;
      }
      this->have_half_ = false;
      this->samples_.push_back(0.5f * (this->half_ + sample));
    } else {
      this->samples_.push_back(sample);
    }
    if (this->samples_.size() == FFT_SIZE) {
      this->compute_spectrum_();
      this->samples_.clear();
    }
  }
}

void AudioSpectrum::compute_spectrum_() {
  const uint32_t rate = this->audio_stream_info_.get_sample_rate() / this->decimation_;
  if (rate != this->sample_rate_) {
    // logarithmically spaced band edges between min_hz and max_hz, each band at least one bin wide
    this->sample_rate_ = rate;
    const float bin_hz = float(rate) / float(FFT_SIZE);
    const float max_hz = std::min(this->max_hz_, float(rate) / 2.0f);
    uint16_t prev = 0;
    for (uint8_t b = 0; b <= this->bands_; b++) {
      float hz = this->min_hz_ * std::pow(max_hz / this->min_hz_, float(b) / float(this->bands_));
      uint16_t bin = std::max<uint16_t>(uint16_t(std::lround(hz / bin_hz)), b == 0 ? 1 : prev + 1);
      this->band_edges_[b] = std::min<uint16_t>(bin, FFT_SIZE / 2);
      prev = this->band_edges_[b];
    }
  }

  for (size_t i = 0; i < FFT_SIZE; i++)
    this->fft_[i] = {this->samples_[i] * this->window_[i], 0.0f};
  fft(this->fft_.data(), FFT_SIZE);

  const float bin_hz = float(rate) / float(FFT_SIZE);
  for (uint8_t b = 0; b < this->bands_; b++) {
    float peak = 0.0f;
    for (uint16_t k = this->band_edges_[b]; k < std::max<uint16_t>(this->band_edges_[b + 1], this->band_edges_[b] + 1);
         k++)
      peak = std::max(peak, std::abs(this->fft_[k]));
    // a full-scale sine gives |X| = N/4 with a Hann window
    float amplitude = peak / (float(FFT_SIZE) / 4.0f);
    float center_hz = float(this->band_edges_[b] + this->band_edges_[b + 1]) * 0.5f * bin_hz;
    float db = 20.0f * std::log10(amplitude + 1e-9f) + TILT_DB_PER_OCTAVE * std::log2(center_hz / 1000.0f);
    float level = std::clamp((db - FLOOR_DB) / -FLOOR_DB, 0.0f, 1.0f);
    this->levels_[b] = std::max(level, this->levels_[b] * RELEASE);
  }
  this->last_analysis_ms_ = millis();
}

float AudioSpectrum::get_level(uint8_t band) const {
  if (band >= this->bands_ || millis() - this->last_analysis_ms_ > STALE_MS)
    return 0.0f;
  return this->levels_[band];
}

}  // namespace esphome::audio_spectrum

#endif
