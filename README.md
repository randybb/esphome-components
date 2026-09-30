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
