# esphome-components

Custom ESPHome components, referenced from device configs via
[`external_components`](https://esphome.io/components/external_components.html)
(`source: github://randybb/esphome-components`) -- no fork of ESPHome itself.

```yaml
external_components:
  - source: github://randybb/esphome-components
    components: [audio_spectrum]
```

## Components

| Component | Description |
|---|---|
| [audio_spectrum](components/audio_spectrum/) | Pass-through speaker that computes a spectrum of the audio for a Winamp style analyzer display |
| [audio](components/audio/) | Temporary copy of ESPHome's `audio` (2026.10.0-dev) whose HTTP reader connects over IPv6 when `network: enable_ipv4: false`. Drop it once esphome/esphome#19750 includes the fix. |
| [speaker](components/speaker/) | Temporary copy of ESPHome's `speaker` (2026.10.0-dev) whose media player skips a playlist item that fails instead of retrying it forever. Drop it once esphome/esphome#19862 is merged. |
