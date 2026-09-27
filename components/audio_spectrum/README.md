# audio_spectrum

Pass-through speaker for ESP32. Audio played to it is forwarded unchanged to
`output_speaker`, and a spectrum of it is computed along the way for a Winamp
style analyzer display.

The FFT (512 points, Hann window, 48 kHz audio decimated to 24 kHz) runs in the
task that feeds the speaker (e.g. the mixer or media player task), not in the
main loop.

## Configuration

```yaml
external_components:
  - source: github://randybb/esphome-components
    components: [audio_spectrum]

speaker:
  - platform: i2s_audio
    id: i2s_speaker
    # ...

  - platform: audio_spectrum
    id: spectrum
    output_speaker: i2s_speaker
    bands: 20
```

Point whatever produces audio (media player, mixer, ...) at `spectrum` instead
of `i2s_speaker`.

| Option | Default | Description |
|---|---|---|
| `output_speaker` | required | Speaker the audio is forwarded to |
| `bands` | `20` | Number of bands, 4-32 |
| `min_frequency` | `60Hz` | Lower edge of the first band |
| `max_frequency` | `16kHz` | Upper edge of the last band, clamped to half the sample rate |
| `sample_rate` | `48000` | Stream sample rate, must match `output_speaker` |
| `bits_per_sample` | `16` | Stream bits per sample, must match `output_speaker` |
| `num_channels` | `1` | Stream channels, must match `output_speaker`; channels are averaged for the analysis |

Bands are spaced logarithmically between `min_frequency` and `max_frequency`.

## Reading the spectrum

- `id(spectrum).get_bands()` returns the number of bands.
- `id(spectrum).get_level(band)` returns the level of a band as 0..1. It returns
  0 if no audio was analysed in the last 250 ms.

Levels cover -60..0 dB with a +3 dB/octave tilt so music looks flatter, and bars
fall off gradually rather than dropping instantly.

```yaml
display:
  - platform: ...
    lambda: |-
      const int n = id(spectrum).get_bands();
      const int w = it.get_width() / n;
      for (int i = 0; i < n; i++) {
        int h = id(spectrum).get_level(i) * it.get_height();
        it.filled_rectangle(i * w, it.get_height() - h, w - 1, h);
      }
```
