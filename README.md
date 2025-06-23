# OpenGL Flight Simulator

![Clang Format](https://github.com/gue-ni/OpenGL_Flightsim/actions/workflows/clang-format.yml/badge.svg)
![Linux CMake Build](https://github.com/gue-ni/OpenGL_Flightsim/actions/workflows/cmake.yml/badge.svg)
![Linux CMake Build](https://github.com/gue-ni/OpenGL_Flightsim/actions/workflows/msbuild.yml/badge.svg)

You can read an article about how it works [here](https://www.jakobmaier.at/posts/flight-simulation/). A windows release is available [here](https://github.com/gue-ni/OpenGL_Flightsim/releases/), a linux build can be found at the latest cmake.yml run [here](https://github.com/gue-ni/OpenGL_Flightsim/actions).

https://user-images.githubusercontent.com/45669207/221434341-9068181d-0ff3-401a-bf34-29b530a92154.mp4

## Controls

The demo supports input with a keyboard and a joystick. With a keyboard, use WASD to control pitch and roll and F and K to increase and decrease thrust. E and Q control the rudder. You can use P to pause and O to toggle the camera.


## Build instructions (Linux)

Install dependencies SDL2, GLEW and OpenGL and build using cmake.

```
$ apt install libsdl2-dev libsdl2-image-dev libglew-dev libgle3-dev libglm-dev cmake
$ mkdir build 
$ cd build
$ cmake ..
$ cmake --build .
```
