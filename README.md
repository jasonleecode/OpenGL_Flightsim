# OpenGL Flight Simulator

![Linux CMake Build](https://github.com/jasonleecode/OpenGL_Flightsim/actions/workflows/cmake.yml/badge.svg)

An OpenGL flight simulator with a glass-cockpit style HUD: primary flight display, radar scope, moving map and a frosted status bar.

## Controls

### Keyboard

| Key | Action |
| --- | --- |
| `W` / `S` | Pitch down / up |
| `A` / `D` | Roll left / right |
| `Q` / `E` | Rudder left / right |
| `J` / `K` | Decrease / increase thrust |
| `N` / `M` | Pitch trim |
| `P` | Pause |
| `O` | Toggle orbit camera (captures the mouse) |
| `I` | Toggle wireframe terrain |
| `F1` | Toggle flight instruments (PFD) |
| `F2` | Toggle radar scope |
| `F3` | Toggle map display |
| `F10` | Cycle camera view: chase / side / above the wing |
| `F11` | Toggle fullscreen |
| `ESC` | Quit |

### Gamepad

| Input | Action |
| --- | --- |
| Left stick | Pitch and roll |
| Right stick | Yaw |
| `LT` / `RT` | Decrease / increase thrust |
| D-pad up / down | Pitch trim |
| Start | Pause |
| Back | Toggle orbit camera |

### Settings

The **Settings** button (top right corner) opens panels for aircraft selection (Fast Jet / Cessna), the pull-up warning threshold (height above ground) and the frosted glass panel background.

### Sound

Drop `engine.wav` and `radio.wav` into `assets/audio/` to enable engine and radio chatter sounds (see `assets/audio/README.txt`).

## Build instructions (Linux)

Install dependencies SDL2, GLEW and GLM and build using cmake.

```
$ apt install libsdl2-dev libglew-dev libglm-dev cmake
$ mkdir build
$ cd build
$ cmake ..
$ cmake --build .
$ ./flightsim
```

## Credits

Based on [OpenGL_Flightsim](https://github.com/gue-ni/OpenGL_Flightsim) by gue-ni. You can read an article about how the flight model works [here](https://www.jakobmaier.at/posts/flight-simulation/).
