# fcfbfarm I/Q WAV format

When `fcfbfarm` runs with `wav_format = iq` in the `[fcfbfarm]` config section, each
UTC-aligned window is written as a **complex float32 WAV** (the channel's baseband
I/Q) instead of the default real int16 audio. This document specifies that format:
the container, the channel layout, the sample convention, the filename template, and
the `auxi` metadata chunk.

It is meant to be a stable contract for downstream tools (e.g. a diversity decoder)
that read these files.

## Why I/Q

The real-audio WAV keeps only `real(baseband)`, which folds the two sidebands
together. The I/Q WAV keeps the full complex baseband, un-normalised, so that:

- both sidebands and the true phase are preserved, and
- for a dual-ADC (diversity) channel, the **relative amplitude and phase between the
  two antennas** are preserved — the information a coherent combiner (maximum-ratio
  combining) needs.

The I channel equals `real(I/Q)`, which is exactly the audio the real-audio path
would have produced; `wav-to-decoder` relies on this to decode an I/Q file with
jt9/wsprd (it feeds them the I channel).

## Container

| Property | Value |
|---|---|
| Format | RIFF/WAVE |
| `wFormatTag` | `3` (`WAVE_FORMAT_IEEE_FLOAT`) |
| Sample type | 32-bit little-endian IEEE-754 float |
| Sample rate | 12000 Hz (complex) |
| Normalisation | none — raw baseband samples |
| Chunk order | `fmt ` (18-byte extended, `cbSize=0`), `fact`, `auxi`, `data` |

The 18-byte `fmt ` chunk and the `fact` chunk (samples-per-channel) are the standard
requirements for a non-PCM WAVE file. Readers that only understand PCM (e.g. Python's
stdlib `wave`) cannot read these files; use a float-aware reader such as
`libsndfile` / `python-soundfile` or `scipy.io.wavfile`.

## Channel layout

Samples are interleaved (channel-minor), antenna-major, **I before Q**:

| ADC mode (`adc=`) | Channels | Interleave |
|---|---|---|
| single (1 = A, or 2 = B) | 2 | `I, Q` |
| dual / diversity (3) | 4 | `I_A, Q_A, I_B, Q_B` |

The channel count is therefore self-describing: 2 ⇒ single ADC, 4 ⇒ dual ADC. There
is no per-file field for the ordering — it is fixed by this specification.

## Sample convention

- The baseband is centred at the channel centre frequency `fc` (the dial): **0 Hz in
  the file corresponds to `fc`**.
- A tone **above** `fc` appears at a **positive** frequency in the I/Q stream.
- `I = real(baseband)`, `Q = imag(baseband)`.

## Filename convention

The path comes from the `[fcfbfarm] savewav =` template. Placeholders:

| Placeholder | Meaning |
|---|---|
| `{date}` | `YYYYMMDD` (window start, UTC) |
| `{timestamp}` | `YYYYMMDD_HHMMSS` (fast modes) / `YYYYMMDD_HHMM` (slow: WSPR, FST4W) |
| `{channel}` | channel `name` |
| `{antenna}` | `A` (adc 1), `B` (adc 2), `AB` (adc 3) |
| `{frequency}` | centre frequency in whole Hz (e.g. `7074000`) |
| `{decoder}` | decoder name |

A leading `~` expands to the user's home directory; parent directories are created
as needed. Example template and a resulting path:

```
savewav = ~/fcfb/iq/{date}/{channel}/{antenna}-{timestamp}.wav
→ /home/<user>/fcfb/iq/20260927/40mF8iqd/AB-20260927_183000.wav
```

The authoritative window time is in the `auxi` chunk (below); the filename is for
human organisation.

## The `auxi` chunk

`auxi` is the metadata chunk SDR tools of the RFSpace/SpectraVue lineage use to carry
what a WAV header cannot. Its body is **68 bytes**, all little-endian, laid out as:

| Offset | Size | Field | Type | fcfbfarm value |
|---:|---:|---|---|---|
| 0 | 16 | `StartTime` | `SYSTEMTIME` | window start, **UTC** |
| 16 | 16 | `StopTime` | `SYSTEMTIME` | window start + period, **UTC** |
| 32 | 4 | `CenterFreq` | `uint32` | channel `fc`, Hz |
| 36 | 4 | `ADFrequency` | `uint32` | file sample rate (12000), Hz |
| 40 | 4 | `IFFrequency` | `uint32` | 0 |
| 44 | 4 | `Bandwidth` | `uint32` | channel occupied bandwidth, Hz |
| 48 | 4 | `IQOffset` | `uint32` | 0 |
| 52 | 16 | `Unused2..5` | `uint32`×4 | 0 |

The chunk is preceded by the usual 8-byte RIFF chunk header: the ASCII id `auxi`
followed by a `uint32` size (`68`).

`SYSTEMTIME` is the Win32 structure — eight consecutive `uint16` little-endian
fields:

| Field | Notes |
|---|---|
| `wYear` | e.g. 2026 |
| `wMonth` | 1–12 |
| `wDayOfWeek` | 0 = Sunday; written for completeness, **ignored on read** |
| `wDay` | 1–31 |
| `wHour` | 0–23 (UTC) |
| `wMinute` | 0–59 |
| `wSecond` | 0–59 |
| `wMilliseconds` | 0–999 |

The window **length** is `StopTime − StartTime`; a length ≥ 60 s marks a slow mode
(WSPR/FST4W). `wav-to-decoder` uses `StartTime` to name the temporary audio WAV it
hands the decoder, and the length to choose the WSJT-X `HHMM` vs `HHMMSS` form.

## Reading the files

- **Decode with jt9/wsprd:** use `wav-to-decoder` (ships with the host binaries). It
  autodetects the format and channel count, extracts the I channel(s) as audio, and
  runs the decoder — emitting `#ANT A`/`#ANT B` for a dual file. See the host app
  [README](../host_app/gofcfb/README.md).
- **Custom DSP:** read the float32 samples with `python-soundfile`, `scipy`, SoX, GNU
  Radio, etc. De-interleave per the channel layout above; reconstruct the complex
  baseband as `I + 1j*Q` per antenna. Read `fc`, the sample rate and the UTC window
  bounds from the `auxi` chunk.

## Notes

- The I/Q files are **not** normalised; if you need a displayable/audio level, scale
  as appropriate for your tool. The relative scaling across channels (and antennas)
  is meaningful and must be preserved by any combiner.
- `wav_format` is independent of `savewav`: without `savewav` the I/Q WAV is a
  temporary file (deleted after decoding); with it, the file is kept at the template
  path.
- The reference reader/writer is package `fcfb` (`host_app/gofcfb/wav.go`).
