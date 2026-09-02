#pragma once

#include <SDL.h>

#include <cstring>
#include <iostream>
#include <string>
#include <vector>

// minimal looping sound player: mixes wav files into a single sdl audio device.
// missing sound files only print a warning, the game runs fine without them.
namespace audio
{

struct Sound {
  std::vector<Uint8> data;  // converted to the device format
  size_t position = 0;
  float volume    = 1.0f;
  bool playing    = false;
};

class Player
{
 public:
  bool init()
  {
    SDL_AudioSpec want{}, got{};
    want.freq     = 44100;
    want.format   = AUDIO_S16SYS;
    want.channels = 2;
    want.samples  = 2048;
    want.callback = &Player::mix_callback;
    want.userdata = this;

    m_device = SDL_OpenAudioDevice(nullptr, 0, &want, &got, 0);
    if (m_device == 0) {
      std::cout << "[audio] failed to open audio device: " << SDL_GetError() << std::endl;
      return false;
    }
    m_spec   = got;
    m_inited = true;
    return true;
  }

  // load a wav file, returns the sound id or -1 if the file is missing/invalid
  int load(const std::string& path, float volume)
  {
    if (!m_inited) return -1;

    SDL_AudioSpec wav_spec;
    Uint8* wav_buffer = nullptr;
    Uint32 wav_length = 0;
    if (SDL_LoadWAV(path.c_str(), &wav_spec, &wav_buffer, &wav_length) == nullptr) {
      std::cout << "[audio] sound file not found: " << path << ", continuing without it" << std::endl;
      return -1;
    }

    SDL_AudioCVT cvt;
    if (SDL_BuildAudioCVT(&cvt, wav_spec.format, wav_spec.channels, wav_spec.freq, m_spec.format, m_spec.channels,
                          m_spec.freq) < 0) {
      std::cout << "[audio] cannot convert " << path << ": " << SDL_GetError() << std::endl;
      SDL_FreeWAV(wav_buffer);
      return -1;
    }

    cvt.len = static_cast<int>(wav_length);
    cvt.buf = static_cast<Uint8*>(SDL_malloc(wav_length * cvt.len_mult));
    std::memcpy(cvt.buf, wav_buffer, wav_length);
    SDL_ConvertAudio(&cvt);
    SDL_FreeWAV(wav_buffer);

    Sound sound;
    sound.data.assign(cvt.buf, cvt.buf + cvt.len_cvt);
    sound.volume = volume;
    SDL_free(cvt.buf);

    m_sounds.push_back(std::move(sound));
    return static_cast<int>(m_sounds.size()) - 1;
  }

  // start looping a sound
  void play(int id)
  {
    if (!valid(id)) return;
    lock();
    m_sounds[id].playing = true;
    unlock();
  }

  void set_volume(int id, float volume)
  {
    if (!valid(id)) return;
    lock();
    m_sounds[id].volume = volume;
    unlock();
  }

  void start() const
  {
    if (m_inited) SDL_PauseAudioDevice(m_device, 0);
  }

  void shutdown()
  {
    if (m_inited) SDL_CloseAudioDevice(m_device);
    m_inited = false;
  }

 private:
  bool valid(int id) const { return m_inited && 0 <= id && id < static_cast<int>(m_sounds.size()); }
  void lock() const { SDL_LockAudioDevice(m_device); }
  void unlock() const { SDL_UnlockAudioDevice(m_device); }

  static void mix_callback(void* userdata, Uint8* stream, int len)
  {
    auto* self = static_cast<Player*>(userdata);
    std::memset(stream, 0, len);  // silence

    for (auto& sound : self->m_sounds) {
      if (!sound.playing || sound.data.empty()) continue;

      size_t remaining = static_cast<size_t>(len);
      Uint8* out       = stream;
      while (remaining > 0) {
        size_t chunk = std::min(remaining, sound.data.size() - sound.position);
        SDL_MixAudioFormat(out, &sound.data[sound.position], self->m_spec.format, static_cast<Uint32>(chunk),
                           static_cast<int>(sound.volume * SDL_MIX_MAXVOLUME));
        sound.position = (sound.position + chunk) % sound.data.size();  // loop
        out += chunk;
        remaining -= chunk;
      }
    }
  }

  SDL_AudioDeviceID m_device = 0;
  SDL_AudioSpec m_spec{};
  std::vector<Sound> m_sounds;
  bool m_inited = false;
};

};  // namespace audio
