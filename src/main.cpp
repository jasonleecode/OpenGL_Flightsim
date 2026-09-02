#define SDL_MAIN_HANDLED
#include <GL/glew.h>
#include <SDL.h>
#include <SDL_opengl.h>

#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

#include "../lib/imgui/imgui.h"
#include "../lib/imgui/imgui_impl_opengl3.h"
#include "../lib/imgui/imgui_impl_sdl2.h"
#include "ai.h"
#include "audio.h"
#include "collider.h"
#include "flightmodel.h"
#include "gfx.h"
#include "instruments.h"
#include "phi.h"
#include "pid.h"
#include "terrain.h"

using std::cout;
using std::endl;
using std::make_shared;
using std::shared_ptr;

std::string USAGE = R"(
Usage:

P       pause game
O       toggle camera
I       toggle wireframe terrain
WASD    control pitch and roll
EQ      control yaw
JK      control thrust
F1      toggle flight instruments
F2      toggle radar
F3      toggle map
F10     cycle camera view (chase / side / above wing)
F11     toggle fullscreen

Gamepad:
left stick      pitch and roll
right stick     yaw
LT/RT           decrease / increase thrust
dpad up/down    pitch trim
start           pause game
back            toggle camera
)";

#define CLIPMAP            1
#define SKYBOX             1
#define SMOOTH_CAMERA      1
#define NPC_AIRCRAFT       0
#define SHOW_MASS_ELEMENTS 0
#define USE_PID            1
#define PS1_RESOLUTION     0
#define DEBUG_INFO         0

/* aircraft models */
enum AircraftModel { MODEL_FAST_JET, MODEL_CESSNA };

float aircraft_cruise_speed(AircraftModel model)
{
  return phi::units::meter_per_second(model == MODEL_CESSNA ? 200.0f : 500.0f);
}

// build an airplane for the given model
Airplane make_aircraft(AircraftModel model)
{
  if (model == MODEL_CESSNA) {
    // airplane mass
    const float mass = 1000.0f;

    // engine
    const float rpm = 2400.0f;
    const float horsepower = 160.0f;
    const float prop_diameter = 1.9f;

    // main wing
    const float total_wing_area = 16.17f;
    const float total_wing_span = 11.00f;
    const float main_wing_span = total_wing_span / 2;
    const float main_wing_area = total_wing_area / 2;
    const float main_wing_chord = main_wing_area / main_wing_span;

    // horizontal tail
    const float elevator_area = 1.35f;
    const float h_tail_area = 2.0f + elevator_area;
    const float h_tail_span = 2.0f;
    const float h_tail_chord = h_tail_area / h_tail_span;

    // vertical tail
    const float v_tail_area = 2.04f;  // modified
    const float v_tail_span = 2.04f;
    const float v_tail_chord = v_tail_area / v_tail_span;

    const float wing_offset = -0.2f;
    const float tail_offset = -4.6f;

    // design coordinates go from the back forwards
    std::vector<phi::inertia::Element> mass_elements = {
        phi::inertia::cube({wing_offset, 0.5f, -2.7f}, {main_wing_chord, 0.10f, main_wing_span}),  // left wing
        phi::inertia::cube({wing_offset, 0.5f, +2.7f}, {main_wing_chord, 0.10f, main_wing_area}),  // right wing
        phi::inertia::cube({tail_offset, -0.1f, 0.0f}, {h_tail_chord, 0.10f, h_tail_span}),        // elevator
        phi::inertia::cube({tail_offset, 0.0f, 0.0f}, {v_tail_chord, v_tail_span, 0.10f}),         // rudder
        phi::inertia::cube({0.0f, 0.0f, 0.0f}, {8.0f, 2.0f, 1.0f}),                                // fuselage
    };

    // individual element mass is proportional to volume
    glm::vec3 center_of_gravity;
    phi::inertia::set_uniform_density(mass_elements, mass);

    // compute inertia tensor
    const auto inertia = phi::inertia::tensor(mass_elements, true, &center_of_gravity);

    auto l_wing_pos = mass_elements[0].offset;
    auto r_wing_pos = mass_elements[1].offset;
    auto h_tail_pos = mass_elements[2].offset;
    auto v_tail_pos = mass_elements[3].offset;

    // static: wings hold pointers to these, they must outlive the aircraft
    static const Airfoil NACA_0012(NACA_0012_data);
    static const Airfoil NACA_2412(NACA_2412_data);

    std::vector<Wing> wings = {
        Wing(&NACA_2412, l_wing_pos, main_wing_area, main_wing_span, phi::UP, 0.10f),  // left wing
        Wing(&NACA_2412, r_wing_pos, main_wing_area, main_wing_span, phi::UP, 0.10f),  // right wing
        Wing(&NACA_0012, h_tail_pos, h_tail_area, h_tail_span, phi::UP, 0.25f),        // horizontal tail
        Wing(&NACA_0012, v_tail_pos, v_tail_area, v_tail_span, phi::RIGHT, 0.25f),     // vertical tail
    };

    return Airplane(mass, inertia, wings, {new PropellerEngine(horsepower, rpm, prop_diameter)}, nullptr);
  }

  const float mass = 10000.0f;
  const float thrust = 75000.0f;

  const float wing_offset = -1.0f;
  const float tail_offset = -6.6f;

  std::vector<phi::inertia::Element> masses = {
      phi::inertia::cube({wing_offset, 0.0f, -2.7f}, {6.96f, 0.10f, 3.50f}, mass * 0.25f),  // left wing
      phi::inertia::cube({wing_offset, 0.0f, +2.7f}, {6.96f, 0.10f, 3.50f}, mass * 0.25f),  // right wing
      phi::inertia::cube({tail_offset, -0.1f, 0.0f}, {6.54f, 0.10f, 2.70f}, mass * 0.1f),   // elevator
      phi::inertia::cube({tail_offset, 0.0f, 0.0f}, {5.31f, 3.10f, 0.10f}, mass * 0.1f),    // rudder
      phi::inertia::cube({0.0f, 0.0f, 0.0f}, {8.0f, 2.0f, 2.0f}, mass * 0.5f),              // fuselage
  };

  const auto inertia = phi::inertia::tensor(masses, true);

  // static: wings hold pointers to these, they must outlive the aircraft
  static const Airfoil NACA_0012(NACA_0012_data);
  static const Airfoil NACA_2412(NACA_2412_data);

  std::vector<Wing> wings = {
      // main wings get a few degrees of incidence, otherwise lift at level attitude
      // is not enough to hold altitude and the plane settles into a steady descent
      Wing({wing_offset, 0.0f, -2.7f}, 6.96f, 2.50f, &NACA_2412, Wing::calc_wing_normal(phi::UP, 1.5f), 0.20f),
      Wing({wing_offset, 0.0f, +2.7f}, 6.96f, 2.50f, &NACA_2412, Wing::calc_wing_normal(phi::UP, 1.5f), 0.20f),
      Wing({tail_offset, -0.1f, 0.0f}, 6.54f, 2.70f, &NACA_0012, phi::UP, 1.0f),     // elevator
      Wing({tail_offset, 0.0f, 0.0f}, 5.31f, 3.10f, &NACA_0012, phi::RIGHT, 0.15f),  // rudder
  };

  return Airplane(mass, inertia, wings, {new SimpleEngine(thrust)}, nullptr);
}

#if PS1_RESOLUTION
constexpr glm::ivec2 RESOLUTION{640, 480};
#else
constexpr glm::ivec2 RESOLUTION{1920, 1080};
#endif

struct Joystick {
  int num_axis{0}, num_hats{0}, num_buttons{0};
  float aileron{0.0f}, elevator{0.0f}, rudder{0.0f}, throttle{0.0f}, trim{0.0f};

  // scale from int16 to -1.0, 1.0
  inline static float scale(int16_t value)
  {
    constexpr int16_t max_value = std::numeric_limits<int16_t>::max();
    return static_cast<float>(value) / static_cast<float>(max_value);
  }
};

struct GameObject {
  gfx::Mesh transform;
  Airplane& airplane;
  // collider::Sphere collider;

  void update(float dt) { transform.set_transform(airplane.position, airplane.rotation); }
};

void get_keyboard_state(Joystick& joystick, phi::Seconds dt);
void poll_gamepad(SDL_GameController* gamepad, Joystick& joystick, phi::Seconds dt);

int main(void)
{
  SDL_Init(SDL_INIT_EVERYTHING);

  SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 4);
  SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
  SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);

  SDL_Window* window = SDL_CreateWindow("Flightsim", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, RESOLUTION.x,
                                        RESOLUTION.y, SDL_WINDOW_OPENGL);

  SDL_GLContext context = SDL_GL_CreateContext(window);
  glewExperimental = GL_TRUE;

  if (GLEW_OK != glewInit()) return -1;

  std::cout << glGetString(GL_VERSION) << std::endl;
  std::cout << glGetString(GL_SHADING_LANGUAGE_VERSION) << std::endl;
  std::cout << glGetString(GL_VENDOR) << std::endl;
  std::cout << glGetString(GL_RENDERER) << std::endl;

  std::cout << USAGE << std::endl;

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();

  // fonts for the instrument panel, fall back to the imgui default font
  instruments::Style instrument_style;
  {
    ImGuiIO& io = ImGui::GetIO();
    instrument_style.font =
        io.Fonts->AddFontFromFileTTF("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf", 16.0f);
    instrument_style.font_big =
        io.Fonts->AddFontFromFileTTF("/usr/share/fonts/truetype/dejavu/DejaVuSansMono-Bold.ttf", 20.0f);
    instrument_style.font_size     = 16.0f;
    instrument_style.font_big_size = 20.0f;
    if (instrument_style.font == nullptr) {
      instrument_style.font      = io.Fonts->AddFontDefault();
      instrument_style.font_size = 13.0f;
    }
    if (instrument_style.font_big == nullptr) {
      instrument_style.font_big      = instrument_style.font;
      instrument_style.font_big_size = instrument_style.font_size + 4.0f;
    }
  }

  glViewport(0, 0, RESOLUTION.x, RESOLUTION.y);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_MULTISAMPLE);
  glEnable(GL_BLEND);
  glBlendFunc(GL_ONE, GL_ONE_MINUS_SRC_ALPHA);

  ImGui_ImplSDL2_InitForOpenGL(window, context);
  ImGui_ImplOpenGL3_Init();

  // the game only needs raw key events, disable text input so an active IME
  // (ibus/fcitx) does not swallow key presses
  SDL_StopTextInput();

  Joystick joystick;
  joystick.throttle = 0.8f;  // start with some thrust, otherwise the plane just decelerates and descends

  // use pid for keyboard control
  PID pitch_control_pid(1.0f, 0.0f, 0.0f);

  int num_joysticks = SDL_NumJoysticks();

  // prefer the game controller api (standardized button/axis mapping) over the
  // raw joystick api, fall back to the raw api for non-mappable devices
  SDL_GameController* gamepad = nullptr;
  for (int i = 0; i < num_joysticks; i++) {
    if (SDL_IsGameController(i)) {
      gamepad = SDL_GameControllerOpen(i);
      if (gamepad != nullptr) {
        std::cout << "found gamepad: " << SDL_GameControllerName(gamepad) << std::endl;
        break;
      }
    }
  }

  bool joystick_control = gamepad == nullptr && num_joysticks > 0;
  if (joystick_control) {
    std::cout << "found " << num_joysticks << " joysticks\n";
    SDL_JoystickEventState(SDL_ENABLE);
    SDL_Joystick* sdl_joystick = SDL_JoystickOpen(0);
    joystick.num_axis = SDL_JoystickNumAxes(sdl_joystick);
    joystick.num_hats = SDL_JoystickNumHats(sdl_joystick);
    joystick.num_buttons = SDL_JoystickNumButtons(sdl_joystick);
    printf("found %d buttons, %d axis\n", joystick.num_buttons, joystick.num_axis);
  }

  // sound effects: engine loop follows the throttle, radio chatter loops quietly.
  // the wav files are optional, missing files only print a warning
  audio::Player audio_player;
  audio_player.init();
  const int engine_sound = audio_player.load("assets/audio/engine.wav", 0.6f);
  const int radio_sound  = audio_player.load("assets/audio/radio.wav", 0.35f);
  audio_player.play(engine_sound);
  audio_player.play(radio_sound);
  audio_player.start();

  gfx::Renderer renderer(RESOLUTION.x, RESOLUTION.y);

  gfx::gl::TextureParams params = {.flip_vertically = true, .texture_mag_filter = GL_LINEAR};
  auto tex = make_shared<gfx::gl::Texture>("assets/textures/f16_256.jpg", params);
  auto texture = make_shared<gfx::Phong>(tex);
  auto obj = gfx::load_obj("assets/models/falcon.obj");
  auto model = std::make_shared<gfx::Geometry>(obj, gfx::Geometry::POS_NORM_UV);

  gfx::Object3D scene;

  gfx::Light sun(gfx::Light::DIRECTIONAL, glm::vec3(1.0f));
  sun.set_position(glm::vec3(-2.0f, 4.0f, -1.0f));
  sun.cast_shadow = false;
  scene.add(&sun);

#if SKYBOX
  const std::string skybox_path = "assets/textures/skybox/1/";
  gfx::Skybox skybox({
      skybox_path + "right.jpg",
      skybox_path + "left.jpg",
      skybox_path + "top.jpg",
      skybox_path + "bottom.jpg",
      skybox_path + "front.jpg",
      skybox_path + "back.jpg",
  });
  skybox.set_scale(glm::vec3(3.0f));
  scene.add(&skybox);
#endif

#if CLIPMAP
  Clipmap clipmap;
  scene.add(&clipmap);
#endif

  // terrain texture for the map instrument, same source and mapping as the terrain shader
  gfx::gl::Texture map_texture(PATH + "texture.png", gfx::gl::TextureParams{.texture_mag_filter = GL_LINEAR});
  const float map_terrain_size = MAX_TILE_SIZE / ZOOM_FACTOR;

  // heightmap for terrain height lookups (used by the pull-up warning)
  int hm_width = 0, hm_height = 0, hm_channels = 0;
  uint8_t* hm_data =
      gfx::gl::Texture::load_image(PATH + "heightmap.png", &hm_width, &hm_height, &hm_channels, false);

  // terrain height at world (x, z), same mapping and scale as the terrain shader
  auto terrain_height = [&](float x, float z) {
    if (hm_data == nullptr) return 0.0f;
    float u = x / map_terrain_size + 0.5f;
    float v = z / map_terrain_size + 0.5f;
    if (u < 0.0f || u >= 1.0f || v < 0.0f || v >= 1.0f) return 0.0f;
    int px = glm::clamp(static_cast<int>(u * hm_width), 0, hm_width - 1);
    int py = glm::clamp(static_cast<int>(v * hm_height), 0, hm_height - 1);
    return 3000.0f * hm_data[(py * hm_width + px) * hm_channels] / 255.0f;
  };

  // low-res texture + fbo for the frosted glass status bar: the framebuffer region
  // behind the bar is blitted into this texture each frame and stretched back for a blur
  const glm::ivec2 glass_tex_size(160, 16);
  gfx::gl::Texture glass_tex;
  glass_tex.bind(0);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, glass_tex_size.x, glass_tex_size.y, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
  glass_tex.set_parameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glass_tex.set_parameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glass_tex.unbind();

  GLuint glass_fbo = 0;
  glGenFramebuffers(1, &glass_fbo);
  glBindFramebuffer(GL_FRAMEBUFFER, glass_fbo);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, glass_tex.id, 0);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  // low-res textures for the frosted glass panel backgrounds, same trick as the status bar.
  // they are blitted into the shared fbo one at a time after the scene is rendered
  auto make_blur_tex = [](int width, int height) {
    GLuint tex = 0;
    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, nullptr);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glBindTexture(GL_TEXTURE_2D, 0);
    return tex;
  };
  const glm::ivec2 pfd_blur_size(96, 79), scope_blur_size(62, 62);  // roughly 1/5 of the panel size
  const GLuint pfd_blur_tex   = make_blur_tex(pfd_blur_size.x, pfd_blur_size.y);
  const GLuint radar_blur_tex = make_blur_tex(scope_blur_size.x, scope_blur_size.y);
  const GLuint map_blur_tex   = make_blur_tex(scope_blur_size.x, scope_blur_size.y);
#if 0
  int width, height, channels;
  const std::string heightmap_path = "assets/textures/terrain/1/heightmap.png";
  uint8_t* data                    = gfx::gl::Texture::load_image(heightmap_path, &width, &height, &channels, 0);
  phi::Heightmap terrain_collider(data, width, height, channels);
#endif

  std::vector<GameObject*> objects;

  glm::vec3 initial_position = glm::vec3(0.0f, 3000.0f, 0.0f);

  AircraftModel aircraft_model = MODEL_FAST_JET;
  std::vector<Airplane> rigid_bodies = {make_aircraft(aircraft_model)};

  GameObject player = {
      .transform = gfx::Mesh(model, texture),
      .airplane = rigid_bodies[0],
  };

  player.airplane.position = initial_position;
  player.airplane.velocity = glm::vec3(aircraft_cruise_speed(aircraft_model), 0.0f, 0.0f);
  scene.add(&player.transform);
  objects.push_back(&player);

#if SHOW_MASS_ELEMENTS
  auto red_texture = make_shared<gfx::Phong>(glm::vec3(1.0f, 0.0f, 0.0f));

  for (int i = 0; i < mass_elements.size(); i++) {
    auto& mass = mass_elements[i];
    auto element = new gfx::Mesh(gfx::make_cube_geometry(1.0f), red_texture);
    element->set_position(mass.offset);
    element->set_scale(mass.size);
    player.transform.add(element);
  }
#endif

#if NPC_AIRCRAFT
  GameObject npc = {.transform = gfx::Mesh(model, texture),
                    .airplane = Airplane(mass, inertia, wings, engine),
                    .collider = collider::Sphere({0.0f, 0.0f, 0.0f}, 15.0f)};

  npc.airplane.position = position - glm::vec3(-100.0f, 0.0f, 10.0f);
  npc.airplane.velocity = glm::vec3(speed, 0.0f, 0.0f);
  scene.add(&npc.transform);
  objects.push_back(&npc);

  auto red = glm::vec3(1.0f, 0.0f, 0.0f);
  gfx::Billboard target_marker(make_shared<gfx::gl::Texture>("assets/textures/sprites/triangle.png"), red);
  target_marker.set_scale(glm::vec3(0.05f));
  target_marker.set_position({0.0f, 10.0f, 0.0f});
  target_marker.transform_flags = OBJ3D_TRANSFORM | OBJ3D_SCALE;
  npc.transform.add(&target_marker);
#endif

#if 1
  float size = 0.1f;
  float projection_distance = 150.0f;
  glm::vec3 green(0.0f, 1.0f, 0.0f);
  gfx::Billboard cross(make_shared<gfx::gl::Texture>("assets/textures/sprites/cross.png"), green);
  cross.set_position(phi::FORWARD * projection_distance);
  cross.set_scale(glm::vec3(size));
  player.transform.add(&cross);

  gfx::Billboard fpm(make_shared<gfx::gl::Texture>("assets/textures/sprites/fpm.png"), green);
  fpm.set_scale(glm::vec3(size));
  player.transform.add(&fpm);
#endif

  gfx::Object3D camera_transform;
  camera_transform.set_position({-25.0f, 5, 0});
  camera_transform.set_rotation({0, glm::radians(-90.0f), 0.0f});
  player.transform.add(&camera_transform);

  gfx::Camera camera(glm::radians(45.0f), (float)RESOLUTION.x / (float)RESOLUTION.y, 1.0f, 150000.0f);
#if SMOOTH_CAMERA
  camera.set_position(player.airplane.position);
  camera.set_rotation({0, glm::radians(-90.0f), 0.0f});
  scene.add(&camera);
#else
  camera_transform.add(&camera);
#endif

  gfx::OrbitController controller(30.0f);

  SDL_Event event;
  bool quit = false, paused = false, orbit = false;
  int view_mode = 0;  // F10 cycles: 0 = chase, 1 = side, 2 = above the wing
  bool show_settings = false;
  bool glass_panels = false;            // frosted glass panel backgrounds instead of solid black
  float warning_altitude = 150.0f;      // pull-up warning below this altitude

  // panel geometry in imgui coordinates (origin top left): fixed sizes, the slots are
  // computed per frame from the current window size so the panels never stretch
  const ImVec2 status_bar_size(760.0f, 62.0f);
  const ImVec2 pfd_win_size(500.0f, 415.0f), pfd_box_size(480.0f, 395.0f);
  const ImVec2 scope_win_size(330.0f, 330.0f), scope_box_size(310.0f, 310.0f);
  const ImVec2 box_offset(10.0f, 10.0f);  // panel box inset within its window

  // current window size, updated on resize events (e.g. F11 fullscreen)
  glm::ivec2 window_size = RESOLUTION;
  bool fullscreen = false;

  // panel visibility, toggled with F1/F2/F3, all visible by default
  bool show_pfd = true, show_radar = true, show_map = true;
  // show/hide animation state, 0 = hidden, 1 = shown
  float pfd_anim = 1.0f, radar_anim = 1.0f, map_anim = 1.0f;
  // current on-screen box positions, updated every frame, used by the blur blits
  ImVec2 pfd_box_cur   = instruments::add(ImVec2(0.0f, RESOLUTION.y - pfd_win_size.y), box_offset);
  ImVec2 radar_box_cur = instruments::add(ImVec2(RESOLUTION.x - scope_win_size.x, RESOLUTION.y - scope_win_size.y),
                                          box_offset);
  ImVec2 map_box_cur   = instruments::add(ImVec2(RESOLUTION.x - scope_win_size.x,
                                                 RESOLUTION.y - 2.0f * scope_win_size.y - 10.0f),
                                          box_offset);

  // rebuild the player aircraft with a different model, keeping position and attitude
  auto switch_aircraft = [&](AircraftModel model) {
    auto& old = rigid_bodies[0];
    for (auto e : old.engines) delete e;

    Airplane next  = make_aircraft(model);
    next.position  = old.position;
    next.rotation  = old.rotation;
    float spd      = glm::length(old.velocity);
    next.velocity  = (spd > 1.0f ? old.velocity / spd : old.forward()) * aircraft_cruise_speed(model);
    aircraft_model = model;
    rigid_bodies[0] = std::move(next);  // move: Wing is not copy-assignable due to its const members
  };
  uint64_t last = 0, now = SDL_GetPerformanceCounter();
  phi::Seconds dt, timer = 0, log_timer = 0, flight_time = 0.0f;
  float fps = 0.0f;

  while (!quit) {
    // delta time in seconds
    last = now;
    now = SDL_GetPerformanceCounter();
    dt = static_cast<phi::Seconds>((now - last) / static_cast<phi::Seconds>(SDL_GetPerformanceFrequency()));
    flight_time += dt;
    dt = std::min(dt, 0.02f);

    if ((timer += dt) >= 1.0f) {
      timer = 0.0f;
      fps = 1.0f / dt;
    }

    while (SDL_PollEvent(&event) != 0) {
      ImGui_ImplSDL2_ProcessEvent(&event);
      switch (event.type) {
        case SDL_QUIT: {
          quit = true;
          break;
        }
        case SDL_MOUSEMOTION: {
          controller.move_mouse(static_cast<float>(event.motion.xrel), static_cast<float>(event.motion.yrel));
          break;
        }
        case SDL_WINDOWEVENT: {
          // keep the viewport, projection and ui layout in sync with the window size
          if (event.window.event == SDL_WINDOWEVENT_SIZE_CHANGED) {
            window_size = glm::ivec2(event.window.data1, event.window.data2);
            renderer.set_size(window_size.x, window_size.y);
            camera.set_aspect(static_cast<float>(window_size.x) / static_cast<float>(window_size.y));
          }
          break;
        }
        case SDL_KEYDOWN: {
          switch (event.key.keysym.sym) {
            case SDLK_ESCAPE: {
              quit = true;
              break;
            }
            case SDLK_p:
              paused = !paused;
              break;

            case SDLK_o:
              orbit = !orbit;
              // only capture the mouse while the orbit camera is active
              SDL_ShowCursor(orbit ? SDL_FALSE : SDL_TRUE);
              SDL_SetRelativeMouseMode(orbit ? SDL_TRUE : SDL_FALSE);
              break;

            case SDLK_i:
#if CLIPMAP
              clipmap.wireframe = !clipmap.wireframe;
#endif
              break;

            case SDLK_F1:
              show_pfd = !show_pfd;
              break;
            case SDLK_F2:
              show_radar = !show_radar;
              break;
            case SDLK_F3:
              show_map = !show_map;
              break;
            case SDLK_F11:
              fullscreen = !fullscreen;
              SDL_SetWindowFullscreen(window, fullscreen ? SDL_WINDOW_FULLSCREEN_DESKTOP : 0);
              break;
            case SDLK_F10:
              view_mode = (view_mode + 1) % 3;
              break;

            default:
              break;
          }
          break;
        }
        case SDL_JOYAXISMOTION: {
          if ((event.jaxis.value < -3200) || (event.jaxis.value > 3200)) {
            uint8_t axis = event.jaxis.axis;
            int16_t value = event.jaxis.value;
            switch (axis) {
              case 0:
                joystick.aileron = std::pow(Joystick::scale(value), 3.0f);
                break;
              case 1:
                joystick.elevator = std::pow(Joystick::scale(value), 3.0f);
                break;

              case 2:
                joystick.throttle = (Joystick::scale(value) + 1.0f) / 2.0f;
                break;

              case 3:
                // ?
                break;

              case 4:
                joystick.rudder = std::pow(Joystick::scale(value), 3.0f);
                break;

              default:
                break;
            }
          }
          break;
        }
        case SDL_MOUSEWHEEL: {
          if (event.wheel.y > 0) {
            controller.radius *= 1.1f;
          } else if (event.wheel.y < 0) {
            controller.radius *= 0.9f;
          }
          break;
        }
        case SDL_CONTROLLERBUTTONDOWN: {
          switch (event.cbutton.button) {
            case SDL_CONTROLLER_BUTTON_START:
              paused = !paused;
              break;
            case SDL_CONTROLLER_BUTTON_BACK:
              orbit = !orbit;
              SDL_ShowCursor(orbit ? SDL_FALSE : SDL_TRUE);
              SDL_SetRelativeMouseMode(orbit ? SDL_TRUE : SDL_FALSE);
              break;
            default:
              break;
          }
          break;
        }
        case SDL_CONTROLLERDEVICEADDED: {
          if (gamepad == nullptr && SDL_IsGameController(event.cdevice.which)) {
            gamepad = SDL_GameControllerOpen(event.cdevice.which);
            if (gamepad != nullptr) {
              std::cout << "gamepad connected: " << SDL_GameControllerName(gamepad) << std::endl;
            }
          }
          break;
        }
        case SDL_CONTROLLERDEVICEREMOVED: {
          if (gamepad != nullptr &&
              SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(gamepad)) == event.cdevice.which) {
            SDL_GameControllerClose(gamepad);
            gamepad = nullptr;
            std::cout << "gamepad disconnected" << std::endl;
          }
          break;
        }
      }
    }

    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplSDL2_NewFrame();
    ImGui::NewFrame();

    ImGuiWindowFlags window_flags = 0;
    window_flags |= ImGuiWindowFlags_NoTitleBar;
    window_flags |= ImGuiWindowFlags_NoMove;
    window_flags |= ImGuiWindowFlags_NoResize;
    window_flags |= ImGuiWindowFlags_NoInputs;

    // flight data shared by all instruments
    auto& ac = player.airplane;

    // pitch and roll from the body axis vectors: unlike euler angles these
    // are independent of the heading and do not flip when flying south
    float pitch_deg = glm::degrees(std::asin(glm::clamp(ac.forward().y, -1.0f, 1.0f)));
    float roll_deg  = glm::degrees(std::atan2(-ac.right().y, ac.up().y));

    glm::vec3 fwd     = ac.forward();
    float heading_deg = std::fmod(glm::degrees(std::atan2(fwd.z, fwd.x)) + 360.0f, 360.0f);

    float ias_kmh        = phi::units::kilometer_per_hour(ac.get_ias());
    float altitude       = ac.get_altitude();
    float vertical_speed = ac.velocity.y;

    // panel slots for the current window size: corner anchored, fixed pixel sizes,
    // so fullscreen just moves the panels to the corners instead of stretching them
    const ImVec2 status_bar_pos((window_size.x - status_bar_size.x) * 0.5f, 15.0f);
    const ImVec2 pfd_slot(0.0f, window_size.y - pfd_win_size.y);
    const ImVec2 radar_slot(window_size.x - scope_win_size.x, window_size.y - scope_win_size.y);
    const ImVec2 map_slot(radar_slot.x, radar_slot.y - scope_win_size.y - 10.0f);

    // frosted glass status bar at the top center of the window
    {
      const ImVec2 bar_size = status_bar_size;
      const ImVec2 bar_pos  = status_bar_pos;

      ImGui::SetNextWindowPos(bar_pos);
      ImGui::SetNextWindowSize(bar_size);
      ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
      ImGui::Begin("StatusBar", nullptr, window_flags);

      ImDrawList* dl  = ImGui::GetWindowDrawList();
      const ImVec2 wp = ImGui::GetWindowPos();

      // heavily downscaled copy of the framebuffer behind the bar, stretched back = blur
      // (the texture content is updated after the 3d scene is rendered, see below)
      dl->AddImageRounded(reinterpret_cast<ImTextureID>(static_cast<intptr_t>(glass_tex.id)), wp,
                          instruments::add(wp, bar_size), ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f),
                          IM_COL32(255, 255, 255, 255), 10.0f);
      dl->AddRectFilled(wp, instruments::add(wp, bar_size), IM_COL32(15, 20, 26, 140), 10.0f);
      dl->AddRect(wp, instruments::add(wp, bar_size), IM_COL32(255, 255, 255, 55), 10.0f, 0, 1.0f);

      const char* labels[] = {"IAS km/h", "ALT m", "HDG", "V/S m/s", "MACH"};
      char values[5][16];
      snprintf(values[0], sizeof(values[0]), "%d", static_cast<int>(std::lround(ias_kmh)));
      snprintf(values[1], sizeof(values[1]), "%d", static_cast<int>(std::lround(altitude)));
      snprintf(values[2], sizeof(values[2]), "%03d", static_cast<int>(std::lround(heading_deg)) % 360);
      snprintf(values[3], sizeof(values[3]), "%+.1f", vertical_speed);
      snprintf(values[4], sizeof(values[4]), "%.2f", ac.get_mach());

      for (int i = 0; i < 5; i++) {
        float cx = wp.x + bar_size.x * (i + 0.5f) / 5.0f;
        instruments::text_centered(dl, instrument_style.font, 12.0f, ImVec2(cx, wp.y + 16.0f), instruments::DIM,
                                   labels[i]);
        instruments::text_centered(dl, instrument_style.font_big, 20.0f, ImVec2(cx, wp.y + 42.0f),
                                   instruments::WHITE, values[i]);
      }

      ImGui::End();
      ImGui::PopStyleColor();
    }

    // settings button in the top right corner
    {
      ImGui::SetNextWindowPos(ImVec2(window_size.x - 110.0f, 10.0f));
      ImGui::SetNextWindowSize(ImVec2(100.0f, 40.0f));
      ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.04f, 0.045f, 0.05f, 0.60f));
      ImGui::Begin("SettingsButton", nullptr,
                   (window_flags & ~ImGuiWindowFlags_NoInputs) | ImGuiWindowFlags_NoFocusOnAppearing);
      ImGui::PushFont(instrument_style.font);
      if (ImGui::Button("Settings", ImVec2(88.0f, 28.0f))) {
        show_settings = !show_settings;
      }
      ImGui::PopFont();
      ImGui::End();
      ImGui::PopStyleColor();
    }

    // floating settings window, one collapsible panel per settings group
    if (show_settings) {
      ImGui::SetNextWindowPos(ImVec2(window_size.x - 320.0f, 60.0f), ImGuiCond_FirstUseEver);
      ImGui::SetNextWindowSize(ImVec2(260.0f, 0.0f));  // auto height
      ImGui::PushFont(instrument_style.font);
      ImGui::Begin("Settings", &show_settings);
      if (ImGui::CollapsingHeader("Aircraft", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (ImGui::RadioButton("Fast Jet", aircraft_model == MODEL_FAST_JET)) {
          switch_aircraft(MODEL_FAST_JET);
        }
        if (ImGui::RadioButton("Cessna", aircraft_model == MODEL_CESSNA)) {
          switch_aircraft(MODEL_CESSNA);
        }
      }
      if (ImGui::CollapsingHeader("Warnings", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SliderFloat("Pull up below (AGL)", &warning_altitude, 50.0f, 3500.0f, "%.0f m");
      }
      if (ImGui::CollapsingHeader("Display", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Checkbox("Frosted glass panels", &glass_panels);
      }
      ImGui::End();
      ImGui::PopFont();
    }

    // pull-up warning, flashing red in the center of the window.
    // triggers on height above ground, the terrain itself is several hundred meters high
    float agl = altitude - terrain_height(ac.position.x, ac.position.z);
    if (agl < warning_altitude && std::fmod(flight_time, 0.8f) < 0.5f) {
      ImGui::SetNextWindowPos(ImVec2(0, 0));
      ImGui::SetNextWindowSize(ImVec2(window_size.x, window_size.y));
      ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));
      ImGui::Begin("Warning", nullptr, window_flags);

      ImDrawList* dl = ImGui::GetWindowDrawList();
      const ImVec2 center(window_size.x * 0.5f, window_size.y * 0.3f);
      instruments::text_centered(dl, instrument_style.font_big, 64.0f, instruments::add(center, ImVec2(3.0f, 3.0f)),
                                 IM_COL32(0, 0, 0, 255), "PULL UP");
      instruments::text_centered(dl, instrument_style.font_big, 64.0f, center, IM_COL32(255, 60, 60, 255),
                                 "PULL UP");

      ImGui::End();
      ImGui::PopStyleColor();
    }

    // show/hide animation for the panels, ease-out cubic slide
    const float anim_step = 3.5f * dt;
    pfd_anim   = glm::clamp(pfd_anim + (show_pfd ? anim_step : -anim_step), 0.0f, 1.0f);
    radar_anim = glm::clamp(radar_anim + (show_radar ? anim_step : -anim_step), 0.0f, 1.0f);
    map_anim   = glm::clamp(map_anim + (show_map ? anim_step : -anim_step), 0.0f, 1.0f);
    auto ease_out = [](float t) { return 1.0f - std::pow(1.0f - t, 3.0f); };
    auto lerp     = [](float a, float b, float t) { return a + (b - a) * t; };

    // hidden panels slide off screen: the pfd to the left, radar and map to the right.
    // the map slides down into the radar slot when the radar is hidden
    const ImVec2 pfd_win_pos(lerp(-pfd_win_size.x - 5.0f, pfd_slot.x, ease_out(pfd_anim)), pfd_slot.y);
    const ImVec2 radar_win_pos(lerp(window_size.x + 5.0f, radar_slot.x, ease_out(radar_anim)), radar_slot.y);
    const ImVec2 map_win_pos(lerp(window_size.x + 5.0f, map_slot.x, ease_out(map_anim)),
                             lerp(map_slot.y, radar_slot.y, ease_out(1.0f - radar_anim)));
    pfd_box_cur   = instruments::add(pfd_win_pos, box_offset);
    radar_box_cur = instruments::add(radar_win_pos, box_offset);
    map_box_cur   = instruments::add(map_win_pos, box_offset);

    // panel box background: solid dark, or frosted glass when enabled in the settings
    auto draw_panel_bg = [&](ImDrawList* dl, const ImVec2& bmin, const ImVec2& bmax, GLuint blur_tex) {
      if (glass_panels) {
        // heavily downscaled copy of the framebuffer behind the box, stretched back = blur
        dl->AddImageRounded(reinterpret_cast<ImTextureID>(static_cast<intptr_t>(blur_tex)), bmin, bmax,
                            ImVec2(0.0f, 1.0f), ImVec2(1.0f, 0.0f), IM_COL32(255, 255, 255, 255), 8.0f);
        dl->AddRectFilled(bmin, bmax, IM_COL32(8, 9, 11, 130), 8.0f);
      } else {
        dl->AddRectFilled(bmin, bmax, IM_COL32(8, 9, 11, 245), 8.0f);
      }
      dl->AddRect(bmin, bmax, IM_COL32(70, 75, 85, 255), 8.0f, 0, 1.5f);
    };

    // instrument panel: just the PFD box in the bottom left corner of the window
    if (pfd_anim > 0.0f) {
      ImGui::SetNextWindowPos(pfd_win_pos);
      ImGui::SetNextWindowSize(pfd_win_size);
      ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));  // transparent, only the box shows
      ImGui::Begin("Instruments", nullptr, window_flags);

      ImDrawList* dl  = ImGui::GetWindowDrawList();
      const ImVec2 wp = ImGui::GetWindowPos();

      // the pfd box
      const ImVec2 box_min = pfd_box_cur;
      const ImVec2 box_max = instruments::add(pfd_box_cur, pfd_box_size);
      draw_panel_bg(dl, box_min, box_max, pfd_blur_tex);

      // flight mode annunciator row (static, for the look)
      instruments::draw_fma(dl, instrument_style, ImVec2(box_min.x + 6.0f, box_min.y + 4.0f),
                            box_max.x - box_min.x - 12.0f, 44.0f);
      dl->AddLine(ImVec2(box_min.x + 2.0f, box_min.y + 54.0f), ImVec2(box_max.x - 2.0f, box_min.y + 54.0f),
                  IM_COL32(90, 95, 105, 255), 1.0f);

      // attitude ball with flanking tapes and the vsi trapezoid
      instruments::draw_attitude_indicator(dl, instrument_style, ImVec2(wp.x + 240.0f, wp.y + 204.0f), 120.0f,
                                           pitch_deg, roll_deg);
      instruments::draw_tape(dl, instrument_style, ImVec2(wp.x + 30.0f, wp.y + 84.0f), ImVec2(48.0f, 240.0f),
                             ias_kmh, 2.0f, 10.0f, 2, false);
      instruments::draw_tape(dl, instrument_style, ImVec2(wp.x + 402.0f, wp.y + 84.0f), ImVec2(48.0f, 240.0f),
                             altitude, 0.25f, 25.0f, 4, true);
      instruments::draw_vsi(dl, instrument_style, ImVec2(wp.x + 468.0f, wp.y + 110.0f), 16.0f, 7.0f, 188.0f,
                            vertical_speed);

      // bottom row: navaid info on the left, heading tape in the middle, icon on the right
      instruments::draw_hdg_tape(dl, instrument_style, ImVec2(wp.x + 120.0f, wp.y + 340.0f), ImVec2(240.0f, 44.0f),
                                 heading_deg);
      instruments::draw_text(dl, instrument_style.font, 13.0f, ImVec2(box_min.x + 18.0f, wp.y + 340.0f),
                             instruments::WHITE, "JAI");
      instruments::draw_text(dl, instrument_style.font, 13.0f, ImVec2(box_min.x + 18.0f, wp.y + 360.0f),
                             instruments::WHITE, "109.90");
      instruments::draw_text(dl, instrument_style.font, 13.0f, ImVec2(box_min.x + 18.0f, wp.y + 380.0f),
                             instruments::WHITE, "7.2 NM");
      instruments::draw_nav_icon(dl, ImVec2(box_max.x - 24.0f, wp.y + 362.0f), 11.0f);

      ImGui::End();
      ImGui::PopStyleColor();
    }

    // textual readouts, top left corner (independent of the pfd panel)
    {
      ImGui::SetNextWindowPos(ImVec2(10, 10));
      ImGui::SetNextWindowSize(ImVec2(190, 175));
      ImGui::SetNextWindowBgAlpha(0.35f);
      ImGui::Begin("Data", nullptr, window_flags);
      ImGui::PushFont(instrument_style.font);
      ImGui::Text("V/S  %+.1f m/s", vertical_speed);
      ImGui::Text("G    %.1f", ac.get_g());
      ImGui::Text("AoA  %.1f", ac.get_aoa());
      ImGui::Text("THR  %.0f %%", ac.throttle * 100.0f);
      ImGui::Text("MACH %.2f", ac.get_mach());
      ImGui::Text("TRIM %.2f", ac.joystick.w);
      ImGui::Text("FPS  %.1f", fps);
      ImGui::PopFont();
      ImGui::End();
    }

    // radar scope in the bottom right corner
    if (radar_anim > 0.0f) {
      ImGui::SetNextWindowPos(radar_win_pos);
      ImGui::SetNextWindowSize(scope_win_size);
      ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));  // transparent, only the box shows
      ImGui::Begin("Radar", nullptr, window_flags);

      ImDrawList* dl  = ImGui::GetWindowDrawList();
      const ImVec2 wp = ImGui::GetWindowPos();

      const ImVec2 box_min = radar_box_cur;
      const ImVec2 box_max = instruments::add(radar_box_cur, scope_box_size);
      draw_panel_bg(dl, box_min, box_max, radar_blur_tex);

      instruments::draw_radar(dl, instrument_style, ImVec2(wp.x + 165.0f, wp.y + 165.0f), 115.0f, flight_time,
                              map_texture.id, ac.position, heading_deg, map_terrain_size);

      // coordinates, top left corner of the box
      char coord[32];
      snprintf(coord, sizeof(coord), "X %07.1f", ac.position.x);
      instruments::draw_text(dl, instrument_style.font, 11.0f, ImVec2(box_min.x + 8.0f, box_min.y + 6.0f),
                             IM_COL32(120, 255, 170, 200), coord);
      snprintf(coord, sizeof(coord), "Z %07.1f", ac.position.z);
      instruments::draw_text(dl, instrument_style.font, 11.0f, ImVec2(box_min.x + 8.0f, box_min.y + 20.0f),
                             IM_COL32(120, 255, 170, 200), coord);

      ImGui::End();
      ImGui::PopStyleColor();
    }

    // map display above the radar, slides down into the radar slot when the radar is hidden
    if (map_anim > 0.0f) {
      ImGui::SetNextWindowPos(map_win_pos);
      ImGui::SetNextWindowSize(scope_win_size);
      ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.0f, 0.0f, 0.0f, 0.0f));  // transparent, only the box shows
      ImGui::Begin("Map", nullptr, window_flags);

      ImDrawList* dl = ImGui::GetWindowDrawList();

      const ImVec2 box_min = map_box_cur;
      const ImVec2 box_max = instruments::add(map_box_cur, scope_box_size);
      draw_panel_bg(dl, box_min, box_max, map_blur_tex);

      instruments::draw_map(dl, instrument_style, box_min, box_max, map_texture.id, player.airplane.position,
                            heading_deg, map_terrain_size);

      ImGui::End();
      ImGui::PopStyleColor();
    }

#if DEBUG_INFO
    auto angular_velocity = glm::degrees(player.airplane.angular_velocity);
    auto attitude = glm::degrees(player.airplane.get_euler_angles());

    ImVec2 size(140, 140);
    ImGui::SetNextWindowPos(ImVec2(window_size.x - size.y - 10.0f, window_size.y - size.y - 10.0f));
    ImGui::SetNextWindowSize(size);
    ImGui::SetNextWindowBgAlpha(0.35f);
    ImGui::Begin("Debug", nullptr, window_flags);
    ImGui::Text("Time:       %.1f", flight_time);
    ImGui::Text("Roll Rate:  %.1f", angular_velocity.x);
    ImGui::Text("Yaw Rate:   %.1f", angular_velocity.y);
    ImGui::Text("Pitch Rate: %.1f", angular_velocity.z);
    ImGui::Text("Roll:       %.1f", attitude.x);
    ImGui::Text("Yaw:        %.1f", attitude.y);
    ImGui::Text("Pitch:      %.1f", attitude.z);
    ImGui::End();
#endif

    get_keyboard_state(joystick, dt);
    if (gamepad != nullptr) {
      poll_gamepad(gamepad, joystick, dt);
    }

    player.airplane.joystick = glm::vec4(joystick.aileron, joystick.rudder, joystick.elevator, joystick.trim);
#if USE_PID
    {
      float max_av = 45.0f;  // deg/s
      float target_av = max_av * joystick.elevator;
      float current_av = glm::degrees(player.airplane.angular_velocity.z);
      player.airplane.joystick.z = pitch_control_pid.calculate(current_av, target_av, dt);
    }
#endif
    player.airplane.throttle = joystick.throttle;

    // engine sound follows the throttle, always audible at idle
    audio_player.set_volume(engine_sound, 0.25f + 0.75f * joystick.throttle);

#if NPC_AIRCRAFT
    target_marker.visible = glm::length(camera.get_world_position() - npc.airplane.position) > 500.0f;
    // fly_towards(npc.airplane, player.airplane.position);
#endif

    if (!paused) {
      phi::step_physics(rigid_bodies, dt);

      for (auto obj : objects) {
        obj->update(dt);
      }
    }

    fpm.set_position(glm::normalize(player.airplane.get_body_velocity()) * projection_distance);

    if (orbit) {
      controller.update(camera, player.airplane.position, dt);
      cross.visible = fpm.visible = false;
    } else if (!paused) {
#if SMOOTH_CAMERA
      auto& rb = player.airplane;
      if (view_mode == 2) {
        // rigidly mounted above and slightly behind the main wing, looking forward over
        // the nose: the whole wing is visible, the tail stays behind the camera.
        // no position smoothing here: the chase lag (~28m at speed) would leave
        // the camera behind the tail
        camera.set_position(rb.position + rb.up() * 4.0f - rb.forward() * 9.0f);
        camera.look_at(rb.position + rb.forward() * 30.0f - rb.up() * 6.0f);
      } else {
        glm::vec3 view_pos = (view_mode == 1) ? rb.position + rb.right() * 28.0f + rb.up() * 3.0f
                                              : rb.position + rb.up() * 4.5f;
        camera.set_position(glm::mix(camera.get_position(), view_pos, dt * 0.035f * rb.get_speed()));
        if (view_mode == 1) {
          camera.look_at(rb.position);
        } else {
          camera.set_rotation_quat(
              glm::mix(camera.get_rotation_quat(), camera_transform.get_world_rotation_quat(), dt * 5.0f));
        }
      }
#else
      camera.set_transform(glm::vec3(0.0f), glm::vec3(0.0f));
#endif
      cross.visible = fpm.visible = (view_mode == 0);  // hud markers only make sense in chase view
    }
    renderer.render(camera, scene);

    // update the frosted glass textures with this frame's scene, each texture gets the
    // framebuffer region behind its panel (gl coordinates, origin at bottom left)
    {
      glBindFramebuffer(GL_READ_FRAMEBUFFER, 0);
      glBindFramebuffer(GL_DRAW_FRAMEBUFFER, glass_fbo);
      auto blit_behind = [&](const ImVec2& pos, const ImVec2& size, GLuint tex, const glm::ivec2& tex_size) {
        const int x0 = static_cast<int>(pos.x);
        const int y0 = static_cast<int>(window_size.y - pos.y - size.y);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);
        glBlitFramebuffer(x0, y0, x0 + static_cast<int>(size.x), y0 + static_cast<int>(size.y), 0, 0, tex_size.x,
                          tex_size.y, GL_COLOR_BUFFER_BIT, GL_LINEAR);
      };
      blit_behind(status_bar_pos, status_bar_size, glass_tex.id, glass_tex_size);
      if (glass_panels) {
        if (pfd_anim > 0.0f) blit_behind(pfd_box_cur, pfd_box_size, pfd_blur_tex, pfd_blur_size);
        if (radar_anim > 0.0f) blit_behind(radar_box_cur, scope_box_size, radar_blur_tex, scope_blur_size);
        if (map_anim > 0.0f) blit_behind(map_box_cur, scope_box_size, map_blur_tex, scope_blur_size);
      }
      glBindFramebuffer(GL_FRAMEBUFFER, 0);
    }

    ImGui::Render();
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
    SDL_GL_SwapWindow(window);
  }
  audio_player.shutdown();
  return 0;
}

inline float move(float value, float factor, float dt) { return glm::clamp(value - factor * dt, -1.0f, 1.0f); }

inline float center(float value, float factor, float dt)
{
  return (value >= 0) ? glm::clamp(value - factor * dt, 0.0f, 1.0f) : glm::clamp(value + factor * dt, -1.0f, 0.0f);
}

void get_keyboard_state(Joystick& joystick, phi::Seconds dt)
{
  const glm::vec3 factor = {3.0f, 0.5f, 1.0f};  // roll, yaw, pitch
  const uint8_t* key_states = SDL_GetKeyboardState(NULL);

  if (key_states[SDL_SCANCODE_A] || key_states[SDL_SCANCODE_LEFT]) {
    joystick.aileron = move(joystick.aileron, +factor.x, dt);
  } else if (key_states[SDL_SCANCODE_D] || key_states[SDL_SCANCODE_RIGHT]) {
    joystick.aileron = move(joystick.aileron, -factor.x, dt);
  } else if (joystick.num_axis <= 0) {
    joystick.aileron = center(joystick.aileron, factor.x, dt);
  }

  if (key_states[SDL_SCANCODE_W] || key_states[SDL_SCANCODE_UP]) {
    joystick.elevator = move(joystick.elevator, +factor.z, dt);
  } else if (key_states[SDL_SCANCODE_S] || key_states[SDL_SCANCODE_DOWN]) {
    joystick.elevator = move(joystick.elevator, -factor.z, dt);
  } else if (joystick.num_axis <= 0) {
    joystick.elevator = center(joystick.elevator, factor.z * 3.0f, dt);
  }

  if (key_states[SDL_SCANCODE_E]) {
    joystick.rudder = move(joystick.rudder, -factor.x, dt);
  } else if (key_states[SDL_SCANCODE_Q]) {
    joystick.rudder = move(joystick.rudder, +factor.x, dt);
  } else if (joystick.num_axis <= 0) {
    joystick.rudder = center(joystick.rudder, factor.z, dt);
  }

  const float throttle_speed = 0.002f;

  if (key_states[SDL_SCANCODE_J]) {
    joystick.throttle = glm::clamp(joystick.throttle - throttle_speed, 0.0f, 1.0f);
  } else if (key_states[SDL_SCANCODE_K]) {
    joystick.throttle = glm::clamp(joystick.throttle + throttle_speed, 0.0f, 1.0f);
  }

  const float trim_speed = 0.002f;
  if (key_states[SDL_SCANCODE_N]) {
    joystick.trim = glm::clamp(joystick.trim - trim_speed, -1.0f, 1.0f);
  } else if (key_states[SDL_SCANCODE_M]) {
    joystick.trim = glm::clamp(joystick.trim + trim_speed, -1.0f, 1.0f);
  }
}

inline float cube(float v) { return v * v * v; }

// read the gamepad state each frame: left stick = pitch/roll, right stick = yaw,
// triggers = thrust, dpad up/down = pitch trim
void poll_gamepad(SDL_GameController* gamepad, Joystick& joystick, phi::Seconds dt)
{
  auto raw_axis = [&](SDL_GameControllerAxis axis) {
    return SDL_GameControllerGetAxis(gamepad, axis) / 32767.0f;
  };

  // scale the value outside the deadzone to the full -1..1 range
  const float deadzone = 0.12f;
  auto axis = [&](SDL_GameControllerAxis a) {
    float v = raw_axis(a);
    if (std::abs(v) < deadzone) return 0.0f;
    return (v - std::copysign(deadzone, v)) / (1.0f - deadzone);
  };

  // stick forward = nose down, stick right = roll right
  joystick.aileron  = cube(-axis(SDL_CONTROLLER_AXIS_LEFTX));
  joystick.elevator = cube(axis(SDL_CONTROLLER_AXIS_LEFTY));
  joystick.rudder   = cube(axis(SDL_CONTROLLER_AXIS_RIGHTX));

  // triggers are unsigned: right = more thrust, left = less
  float right_trigger = (raw_axis(SDL_CONTROLLER_AXIS_TRIGGERRIGHT) + 1.0f) * 0.5f;
  float left_trigger  = (raw_axis(SDL_CONTROLLER_AXIS_TRIGGERLEFT) + 1.0f) * 0.5f;
  joystick.throttle   = glm::clamp(joystick.throttle + (right_trigger - left_trigger) * dt * 0.5f, 0.0f, 1.0f);

  const float trim_speed = 0.5f;
  if (SDL_GameControllerGetButton(gamepad, SDL_CONTROLLER_BUTTON_DPAD_UP)) {
    joystick.trim = glm::clamp(joystick.trim + trim_speed * dt, -1.0f, 1.0f);
  } else if (SDL_GameControllerGetButton(gamepad, SDL_CONTROLLER_BUTTON_DPAD_DOWN)) {
    joystick.trim = glm::clamp(joystick.trim - trim_speed * dt, -1.0f, 1.0f);
  }
}
