#include "SpatialAudio.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstring>

namespace {
constexpr int kSamples = 256, kFft = 512;
constexpr float kSampleRate = 16000, kSoundSpeed = 343;
constexpr float kMicX[4]{0.113f, -0.036f, -0.076f, -0.113f};
constexpr int kPairs[6][2]{{0,1},{0,2},{0,3},{1,2},{1,3},{2,3}};
constexpr float kPi = 3.14159265358979323846f;
using Spectrum = std::array<std::complex<float>, kFft>;

void fft(Spectrum& data, bool inverse) {
  int j = 0;
  for (int i = 1; i < kFft; ++i) {
    int bit = kFft >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) std::swap(data[size_t(i)], data[size_t(j)]);
  }
  for (int length = 2; length <= kFft; length <<= 1) {
    const float angle = (inverse ? 2 : -2) * kPi / length;
    const std::complex<float> root(std::cos(angle), std::sin(angle));
    for (int i = 0; i < kFft; i += length) {
      std::complex<float> step(1, 0);
      for (int k = 0; k < length / 2; ++k) {
        const auto even = data[size_t(i + k)];
        const auto odd = data[size_t(i + k + length / 2)] * step;
        data[size_t(i + k)] = even + odd;
        data[size_t(i + k + length / 2)] = even - odd;
        step *= root;
      }
    }
  }
  if (inverse) for (auto& sample : data) sample /= float(kFft);
}

float correlationAt(const Spectrum& spectrum, float lag) {
  while (lag < 0) lag += kFft;
  while (lag >= kFft) lag -= kFft;
  const int left = int(lag), right = (left + 1) % kFft;
  const float fraction = lag - left;
  return spectrum[size_t(left)].real() * (1 - fraction) +
         spectrum[size_t(right)].real() * fraction;
}

float sampleAt(const std::array<float, kSamples>& channel, float index) {
  const int left = int(std::floor(index));
  if (left < 0 || left >= kSamples) return 0;
  const int right = std::min(kSamples - 1, left + 1);
  const float fraction = index - left;
  return channel[size_t(left)] * (1 - fraction) + channel[size_t(right)] * fraction;
}
}

SpatialAudioResult SpatialAudio::process(const QByteArray& pcm32, unsigned channelMask) {
  SpatialAudioResult result;
  if (pcm32.size() != kSamples * 4 * 4 || !(channelMask & 15u)) return result;
  std::array<std::array<float, kSamples>, 4> channels{};
  Spectrum spectra[4]{};
  int valid = 0;
  double energy = 0;
  for (int channel = 0; channel < 4; ++channel) {
    if (!(channelMask & (1u << channel))) continue;
    ++valid;
    double mean = 0;
    for (int i = 0; i < kSamples; ++i) {
      qint32 sample;
      std::memcpy(&sample, pcm32.constData() + (i * 4 + channel) * 4, sizeof(sample));
      channels[size_t(channel)][size_t(i)] = float(double(sample) / 2147483648.0);
      mean += channels[size_t(channel)][size_t(i)];
      energy += double(channels[size_t(channel)][size_t(i)]) * channels[size_t(channel)][size_t(i)];
    }
    mean /= kSamples;
    for (int i = 0; i < kSamples; ++i) {
      const float window = 0.5f - 0.5f * std::cos(2 * kPi * i / (kSamples - 1));
      spectra[channel][size_t(i)] = (channels[size_t(channel)][size_t(i)] - float(mean)) * window;
    }
    fft(spectra[channel], false);
  }
  result.rms = valid ? float(std::sqrt(energy / (valid * kSamples))) : 0;
  Spectrum correlations[6]{};
  for (int pair = 0; pair < 6; ++pair) {
    const int a = kPairs[pair][0], b = kPairs[pair][1];
    if (!(channelMask & (1u << a)) || !(channelMask & (1u << b))) continue;
    for (int bin = 0; bin < kFft; ++bin) {
      const int folded = std::min(bin, kFft - bin);
      const float hz = folded * kSampleRate / kFft;
      if (hz < 120 || hz > 4200) continue;
      const auto cross = spectra[a][size_t(bin)] * std::conj(spectra[b][size_t(bin)]);
      const float magnitude = std::abs(cross);
      if (magnitude > 1e-8f) correlations[pair][size_t(bin)] = cross / magnitude;
    }
    fft(correlations[pair], true);
  }
  float scores[181]{};
  int bestBin = 90;
  float best = -1e9f;
  for (int bin = 0; bin < 181; ++bin) {
    const float sine = std::sin((bin - 90) * kPi / 180);
    float sum = 0;
    int pairs = 0;
    for (int pair = 0; pair < 6; ++pair) {
      const int a = kPairs[pair][0], b = kPairs[pair][1];
      if (!(channelMask & (1u << a)) || !(channelMask & (1u << b))) continue;
      const float lag = (kMicX[b] - kMicX[a]) * sine * kSampleRate / kSoundSpeed;
      sum += correlationAt(correlations[pair], lag);
      ++pairs;
    }
    scores[bin] = pairs ? sum / pairs : 0;
    if (scores[bin] > best) { best = scores[bin]; bestBin = bin; }
  }
  float second = -1e9f;
  for (int bin = 0; bin < 181; ++bin)
    if (std::abs(bin - bestBin) > 20) second = std::max(second, scores[bin]);
  result.confidence = std::clamp(best - second, 0.0f, 1.0f);
  if (result.rms >= 0.0018f && valid >= 2 && result.confidence >= 0.015f) {
    const float candidate = float(bestBin - 90);
    smoothedAzimuth_ = haveAzimuth_ ? smoothedAzimuth_ * 0.72f + candidate * 0.28f : candidate;
    haveAzimuth_ = true;
  }
  result.azimuthDeg = smoothedAzimuth_;
  result.beamPcm16.resize(kSamples * 2);
  const float center = (kMicX[0] + kMicX[1] + kMicX[2] + kMicX[3]) / 4;
  const float sine = std::sin(result.azimuthDeg * kPi / 180);
  for (int i = 0; i < kSamples; ++i) {
    float sum = 0;
    for (int channel = 0; channel < 4; ++channel) {
      if (!(channelMask & (1u << channel))) continue;
      const float shift = (kMicX[channel] - center) * sine * kSampleRate / kSoundSpeed;
      sum += sampleAt(channels[size_t(channel)], i - shift);
    }
    const qint16 sample = qint16(std::lround(std::clamp(sum / valid * 1.25f, -0.999f, 0.999f) * 32767));
    std::memcpy(result.beamPcm16.data() + i * 2, &sample, sizeof(sample));
  }
  return result;
}
