#pragma once

#include <cmath>
#include <cstdio>

#include <glm/glm.hpp>

#include "../lib/imgui/imgui.h"

// Flight instrument widgets drawn with ImGui draw lists, replicating a modern PFD.
// Screen coordinates: the y axis points down.
namespace instruments
{

constexpr float TWO_PI = 6.28318530718f;

struct Style {
  ImFont* font     = nullptr;  // labels and ticks
  ImFont* font_big = nullptr;  // digital readouts
  float font_size     = 15.0f;
  float font_big_size = 19.0f;
};

const ImU32 WHITE = IM_COL32(235, 240, 245, 255);
const ImU32 GREEN = IM_COL32(30, 230, 130, 255);  // digital readouts and pointers
const ImU32 LIME  = IM_COL32(190, 255, 80, 255);  // fixed aircraft symbols
const ImU32 DIM   = IM_COL32(150, 160, 170, 255);
const ImU32 TAPE  = IM_COL32(80, 83, 90, 235);    // tape background

static inline ImVec2 add(const ImVec2& a, const ImVec2& b) { return ImVec2(a.x + b.x, a.y + b.y); }
static inline ImVec2 sub(const ImVec2& a, const ImVec2& b) { return ImVec2(a.x - b.x, a.y - b.y); }
static inline ImVec2 mul(const ImVec2& v, float s) { return ImVec2(v.x * s, v.y * s); }
static inline float len(const ImVec2& v) { return std::sqrt(v.x * v.x + v.y * v.y); }

// rotate v by angle (radians), clockwise on screen
static inline ImVec2 rot(const ImVec2& v, float angle)
{
  float s = std::sin(angle), c = std::cos(angle);
  return ImVec2(v.x * c - v.y * s, v.x * s + v.y * c);
}

static inline ImVec2 text_size(ImFont* font, float size, const char* text)
{
  return font ? font->CalcTextSizeA(size, FLT_MAX, 0.0f, text) : ImGui::CalcTextSize(text);
}

static inline void draw_text(ImDrawList* dl, ImFont* font, float size, const ImVec2& pos, ImU32 col,
                             const char* text)
{
  if (font) {
    dl->AddText(font, size, pos, col, text);
  } else {
    dl->AddText(pos, col, text);
  }
}

// draw text centered at pos
static inline void text_centered(ImDrawList* dl, ImFont* font, float size, const ImVec2& pos, ImU32 col,
                                 const char* text)
{
  ImVec2 ts = text_size(font, size, text);
  draw_text(dl, font, size, ImVec2(pos.x - ts.x * 0.5f, pos.y - ts.y * 0.5f), col, text);
}

// digital readout box with centered text
static inline void readout_box(ImDrawList* dl, const Style& style, const ImVec2& center, const char* text, ImU32 col)
{
  ImVec2 ts = text_size(style.font_big, style.font_big_size, text);
  ImVec2 box_min(center.x - ts.x * 0.5f - 8.0f, center.y - ts.y * 0.5f - 4.0f);
  ImVec2 box_max(center.x + ts.x * 0.5f + 8.0f, center.y + ts.y * 0.5f + 4.0f);
  dl->AddRectFilled(box_min, box_max, IM_COL32(0, 0, 0, 235), 3.0f);
  dl->AddRect(box_min, box_max, col, 3.0f, 0, 1.5f);
  draw_text(dl, style.font_big, style.font_big_size, ImVec2(center.x - ts.x * 0.5f, center.y - ts.y * 0.5f), col,
            text);
}

// fill the part of the disk (c, r) where dot(x - p, n) <= 0, n must be normalized
static void fill_disk_halfplane(ImDrawList* dl, const ImVec2& c, float r, const ImVec2& p, const ImVec2& n, ImU32 col)
{
  float d = (c.x - p.x) * n.x + (c.y - p.y) * n.y;
  if (d >= r) return;
  if (d <= -r) {
    dl->AddCircleFilled(c, r, col, 64);
    return;
  }

  const ImVec2 t(-n.y, n.x);
  const ImVec2 m(c.x - n.x * d, c.y - n.y * d);  // point on the line closest to c
  const float h = std::sqrt(r * r - d * d);
  const ImVec2 i1(m.x + t.x * h, m.y + t.y * h);
  const ImVec2 i2(m.x - t.x * h, m.y - t.y * h);
  const ImVec2 opp(c.x - n.x * r, c.y - n.y * r);  // point on the circle deepest in the kept half

  float a1 = std::atan2(i1.y - c.y, i1.x - c.x);
  float a2 = std::atan2(i2.y - c.y, i2.x - c.x);
  float am = std::atan2(opp.y - c.y, opp.x - c.x);

  // sweep from a1 to a2 through am
  float da = std::fmod(a2 - a1 + TWO_PI, TWO_PI);
  float dm = std::fmod(am - a1 + TWO_PI, TWO_PI);
  if (dm > da) da -= TWO_PI;

  const int SEG = 40;
  ImVec2 pts[SEG + 2];
  int npts    = 0;
  pts[npts++] = i1;
  for (int i = 1; i < SEG; i++) {
    float a     = a1 + da * static_cast<float>(i) / static_cast<float>(SEG);
    pts[npts++] = ImVec2(c.x + r * std::cos(a), c.y + r * std::sin(a));
  }
  pts[npts++] = i2;
  dl->AddConvexPolyFilled(pts, npts, col);
}

// draw only the part of segment a-b that lies inside the disk (c, r)
static void line_inside_disk(ImDrawList* dl, const ImVec2& a, const ImVec2& b, const ImVec2& c, float r, ImU32 col,
                             float th)
{
  float dx = b.x - a.x, dy = b.y - a.y;
  float fx = a.x - c.x, fy = a.y - c.y;
  float A = dx * dx + dy * dy;
  if (A < 1e-6f) return;
  float Bq   = 2.0f * (fx * dx + fy * dy);
  float C    = fx * fx + fy * fy - r * r;
  float t0   = 0.0f, t1 = 1.0f;
  float disc = Bq * Bq - 4.0f * A * C;
  if (disc > 0.0f) {
    float s  = std::sqrt(disc);
    float ta = (-Bq - s) / (2.0f * A), tb = (-Bq + s) / (2.0f * A);
    if (ta > tb) {
      float tmp = ta;
      ta        = tb;
      tb        = tmp;
    }
    t0 = (ta > t0) ? ta : t0;
    t1 = (tb < t1) ? tb : t1;
  } else if (C > 0.0f) {
    return;  // fully outside
  }
  if (t0 >= t1) return;
  dl->AddLine(ImVec2(a.x + dx * t0, a.y + dy * t0), ImVec2(a.x + dx * t1, a.y + dy * t1), col, th);
}

// artificial horizon: pitch in degrees (positive = nose up), roll in degrees (positive = right wing down)
inline void draw_attitude_indicator(ImDrawList* dl, const Style& style, const ImVec2& center, float radius,
                                    float pitch_deg, float roll_deg)
{
  const ImU32 sky_col    = IM_COL32(60, 140, 210, 255);
  const ImU32 ground_col = IM_COL32(145, 92, 42, 255);

  const float px_per_deg = radius / 45.0f;
  const float angle      = -glm::radians(roll_deg);  // the horizon appears to roll opposite to the aircraft
  const float B          = radius * 2.0f;

  const ImVec2 n = rot(ImVec2(0.0f, -1.0f), angle);  // up direction (towards the sky)
  const ImVec2 t = rot(ImVec2(1.0f, 0.0f), angle);   // horizon tangent

  // point on the horizon line, shifted down when pitching up
  const ImVec2 p = add(center, rot(ImVec2(0.0f, pitch_deg * px_per_deg), angle));

  // fixed tick ring around the ball
  for (int a = 0; a < 360; a += 10) {
    ImVec2 d   = rot(ImVec2(0.0f, -1.0f), glm::radians(static_cast<float>(a)));
    bool major = (a % 30 == 0);
    dl->AddLine(add(center, mul(d, radius + 3.0f)), add(center, mul(d, radius + (major ? 12.0f : 7.0f))), WHITE,
                1.2f);
  }

  // the ball
  dl->AddCircleFilled(center, radius, sky_col, 64);
  fill_disk_halfplane(dl, center, radius, p, n, ground_col);
  line_inside_disk(dl, sub(p, mul(t, B)), add(p, mul(t, B)), center, radius, WHITE, 2.0f);

  // pitch ladder, lines every 10 degrees up to +/- 30
  for (int a = -30; a <= 30; a += 10) {
    if (a == 0) continue;
    float w  = (std::abs(a) % 20 == 0) ? radius * 0.30f : radius * 0.17f;
    ImVec2 l = add(p, mul(n, static_cast<float>(a) * px_per_deg));
    line_inside_disk(dl, sub(l, mul(t, w)), add(l, mul(t, w)), center, radius, WHITE, 1.5f);

    char buf[8] = {};
    snprintf(buf, sizeof(buf), "%d", std::abs(a));
    for (float side : {-1.0f, 1.0f}) {
      ImVec2 tp = add(l, mul(t, side * (w + radius * 0.13f)));
      if (len(sub(tp, center)) < radius - style.font_size) {
        text_centered(dl, style.font, style.font_size, tp, WHITE, buf);
      }
    }
  }

  // roll scale on the rim, rotates with the ball
  const int roll_ticks[] = {-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60};
  for (int a : roll_ticks) {
    bool major = (a % 30 == 0);
    ImVec2 d   = rot(ImVec2(0.0f, -1.0f), angle + glm::radians(static_cast<float>(a)));
    dl->AddLine(add(center, mul(d, radius - 2.0f)), add(center, mul(d, radius - (major ? 15.0f : 9.0f))), WHITE,
                major ? 2.5f : 1.5f);
  }

  // fixed roll pointer at the top
  const ImVec2 tri[3] = {add(center, ImVec2(-7.0f, -radius + 3.0f)), add(center, ImVec2(7.0f, -radius + 3.0f)),
                         add(center, ImVec2(0.0f, -radius + 17.0f))};
  dl->AddConvexPolyFilled(tri, 3, LIME);

  // fixed side wedges at the horizon line
  for (float side : {-1.0f, 1.0f}) {
    ImVec2 w = add(center, ImVec2(side * (radius + 16.0f), 0.0f));
    const ImVec2 wedge[3] = {w, add(w, ImVec2(-side * 9.0f, -6.0f)), add(w, ImVec2(-side * 9.0f, 6.0f))};
    dl->AddConvexPolyFilled(wedge, 3, WHITE);
  }

  // fixed aircraft symbol, horizontal bars with upturned outer ends
  for (float side : {-1.0f, 1.0f}) {
    ImVec2 inner = add(center, ImVec2(side * radius * 0.10f, 0.0f));
    ImVec2 outer = add(center, ImVec2(side * radius * 0.42f, 0.0f));
    dl->AddLine(inner, outer, LIME, 3.5f);
    dl->AddLine(outer, add(outer, ImVec2(0.0f, -radius * 0.08f)), LIME, 3.5f);
  }
  dl->AddCircleFilled(center, 3.0f, LIME, 12);
}

// vertical moving tape (for speed, altitude, ...), value is centered and the scale moves
inline void draw_tape(ImDrawList* dl, const Style& style, const ImVec2& pos, const ImVec2& size, float value,
                      float px_per_unit, float minor_step, int label_every, bool ticks_on_left)
{
  const ImVec2 vmin     = pos;
  const ImVec2 vmax     = add(pos, size);
  const float cy        = pos.y + size.y * 0.5f;
  const float tick_edge = ticks_on_left ? vmin.x : vmax.x;
  const float tick_dir  = ticks_on_left ? 1.0f : -1.0f;
  const float fs        = style.font_size - 3.0f;

  dl->AddRectFilled(vmin, vmax, TAPE);
  dl->PushClipRect(vmin, vmax, true);

  const float half_range = size.y * 0.5f / px_per_unit;
  const float first      = std::floor((value - half_range) / minor_step) * minor_step;
  const int count        = static_cast<int>(std::ceil(2.0f * half_range / minor_step)) + 1;

  for (int i = 0; i <= count; i++) {
    float v = first + static_cast<float>(i) * minor_step;
    if (v < 0.0f) continue;

    float y    = cy - (v - value) * px_per_unit;
    bool major = (i % label_every == 0);
    float tl   = major ? 12.0f : 7.0f;
    dl->AddLine(ImVec2(tick_edge, y), ImVec2(tick_edge + tick_dir * tl, y), WHITE, major ? 2.0f : 1.0f);

    if (major) {
      char buf[16] = {};
      snprintf(buf, sizeof(buf), "%d", static_cast<int>(std::lround(v)));
      ImVec2 ts = text_size(style.font, fs, buf);
      float tx  = ticks_on_left ? vmin.x + tl + 4.0f : vmax.x - tl - 4.0f - ts.x;
      draw_text(dl, style.font, fs, ImVec2(tx, y - ts.y * 0.5f), WHITE, buf);
    }
  }

  dl->PopClipRect();
  dl->AddRect(vmin, vmax, DIM, 0.0f, 0, 1.0f);

  // digital readout box at the center with a pointer towards the tape edge
  char buf[16] = {};
  snprintf(buf, sizeof(buf), "%d", static_cast<int>(std::lround(value)));
  readout_box(dl, style, ImVec2(pos.x + size.x * 0.5f, cy), buf, GREEN);
  float px   = ticks_on_left ? vmin.x - 4.0f : vmax.x + 4.0f;
  float pdir = ticks_on_left ? -7.0f : 7.0f;
  const ImVec2 tri[3] = {ImVec2(px + pdir, cy), ImVec2(px, cy - 6.0f), ImVec2(px, cy + 6.0f)};
  dl->AddConvexPolyFilled(tri, 3, GREEN);
}

// trapezoidal vertical speed indicator, right edge vertical, wider at the top,
// zero in the middle, +/- max_vs at the ends
inline void draw_vsi(ImDrawList* dl, const Style& style, const ImVec2& top_left, float top_width,
                     float bottom_width, float height, float vs, float max_vs = 20.0f)
{
  const float x = top_left.x, y = top_left.y;
  const float cy = y + height * 0.5f;

  // left edge x coordinate at a given y
  auto left_x = [&](float yy) { return x + (top_width - bottom_width) * (yy - y) / height; };

  const ImVec2 poly[4] = {ImVec2(x, y), ImVec2(x + top_width, y), ImVec2(x + top_width, y + height),
                          ImVec2(left_x(y + height), y + height)};
  dl->AddConvexPolyFilled(poly, 4, TAPE);
  for (int i = 0; i < 4; i++) {
    dl->AddLine(poly[i], poly[(i + 1) % 4], DIM, 1.0f);
  }

  // zero line
  dl->AddLine(ImVec2(left_x(cy), cy), ImVec2(x + top_width, cy), WHITE, 1.5f);

  // ticks and labels at +/- max_vs and +/- max_vs/2
  const float fs = style.font_size - 4.0f;
  for (float f : {-1.0f, -0.5f, 0.5f, 1.0f}) {
    float yy   = cy - f * height * 0.5f;
    float lx   = left_x(yy);
    bool major = (f == -1.0f || f == 1.0f);
    dl->AddLine(ImVec2(lx, yy), ImVec2(lx + (major ? 7.0f : 5.0f), yy), WHITE, 1.5f);

    if (major) {
      char buf[8] = {};
      snprintf(buf, sizeof(buf), "%d", static_cast<int>(std::lround(max_vs)));
      ImVec2 ts = text_size(style.font, fs, buf);
      // right-aligned inside the trapezoid, next to its vertical right edge
      draw_text(dl, style.font, fs, ImVec2(x + top_width - ts.x - 3.0f, yy - ts.y * 0.5f), WHITE, buf);
    }
  }

  // green needle pointing at the slanted left edge
  float vy = cy - glm::clamp(vs / max_vs, -1.0f, 1.0f) * height * 0.5f;
  float lx = left_x(vy);
  const ImVec2 tri[3] = {ImVec2(lx, vy), ImVec2(lx + 11.0f, vy - 5.0f), ImVec2(lx + 11.0f, vy + 5.0f)};
  dl->AddConvexPolyFilled(tri, 3, GREEN);
}

// horizontal heading tape, heading in degrees (0..360, increasing with right turns)
inline void draw_hdg_tape(ImDrawList* dl, const Style& style, const ImVec2& pos, const ImVec2& size,
                          float heading_deg)
{
  const ImVec2 vmin = pos;
  const ImVec2 vmax = add(pos, size);
  const float cx    = pos.x + size.x * 0.5f;

  const float px_per_deg = size.x / 70.0f;  // show +/- 35 degrees

  dl->AddRectFilled(vmin, vmax, TAPE);
  dl->PushClipRect(vmin, vmax, true);

  const float view = 40.0f;
  const int first  = static_cast<int>(std::floor((heading_deg - view) / 5.0f)) * 5;
  for (int a = first; a <= static_cast<int>(heading_deg + view); a += 5) {
    // wrap the relative angle to [-180, 180)
    float rel = std::fmod(static_cast<float>(a) - heading_deg + 540.0f, 360.0f) - 180.0f;
    float x   = cx + rel * px_per_deg;

    int h          = ((a % 360) + 360) % 360;
    bool cardinal  = (h % 90 == 0);
    bool labeled   = (h % 10 == 0);
    float tl       = cardinal ? 13.0f : (labeled ? 10.0f : 6.0f);
    ImU32 tick_col = (h == 0) ? IM_COL32(255, 80, 80, 255) : WHITE;
    dl->AddLine(ImVec2(x, vmin.y), ImVec2(x, vmin.y + tl), tick_col, labeled ? 2.0f : 1.0f);

    if (labeled) {
      char buf[4] = {};
      if (cardinal) {
        const char* letters = "NESW";
        snprintf(buf, sizeof(buf), "%c", letters[h / 90]);
      } else {
        snprintf(buf, sizeof(buf), "%d", h / 10);
      }
      text_centered(dl, style.font, style.font_size, ImVec2(x, vmin.y + tl + style.font_size * 0.7f), tick_col, buf);
    }
  }

  dl->PopClipRect();
  dl->AddRect(vmin, vmax, DIM, 0.0f, 0, 1.0f);

  // green caret and stem at the center
  const ImVec2 tri[3] = {ImVec2(cx - 6.0f, vmax.y + 1.0f), ImVec2(cx + 6.0f, vmax.y + 1.0f),
                         ImVec2(cx, vmax.y - 7.0f)};
  dl->AddConvexPolyFilled(tri, 3, GREEN);
  dl->AddLine(ImVec2(cx, vmin.y + 6.0f), ImVec2(cx, vmax.y - 7.0f), GREEN, 2.0f);
}

// flight mode annunciator row: five columns, static texts replicating the reference
inline void draw_fma(ImDrawList* dl, const Style& style, const ImVec2& pos, float width, float height)
{
  const char* cols[5][3] = {
      {"SPEED", nullptr, nullptr}, {"G/S", "ALT", nullptr}, {"LOC*", nullptr, nullptr},
      {"CAT 3", "SINGLE", nullptr}, {"AP1", "1 FD 2", "A/THR"},
  };
  const bool green[5] = {true, true, true, false, false};

  const float col_w = width / 5.0f;
  const float fs    = 13.0f;

  for (int i = 0; i < 5; i++) {
    if (i > 0) {
      dl->AddLine(ImVec2(pos.x + col_w * i, pos.y), ImVec2(pos.x + col_w * i, pos.y + height),
                  IM_COL32(90, 95, 105, 255), 1.0f);
    }

    float cx   = pos.x + col_w * (i + 0.5f);
    ImU32 col  = green[i] ? GREEN : WHITE;
    int n      = 0;
    while (cols[i][n] != nullptr) n++;
    float y = pos.y + (height - n * (fs + 2.0f)) * 0.5f;

    for (int j = 0; j < n; j++) {
      ImVec2 tp(cx, y + fs * 0.5f);
      text_centered(dl, style.font, fs, tp, col, cols[i][j]);
      if (i == 1 && j == 1) {  // box around "ALT"
        ImVec2 ts = text_size(style.font, fs, cols[i][j]);
        dl->AddRect(ImVec2(tp.x - ts.x * 0.5f - 4.0f, tp.y - ts.y * 0.5f - 2.0f),
                    ImVec2(tp.x + ts.x * 0.5f + 4.0f, tp.y + ts.y * 0.5f + 2.0f), GREEN, 2.0f, 0, 1.2f);
      }
      y += fs + 2.0f;
    }
  }
}

// small compass rose icon, bottom right corner of the pfd
inline void draw_nav_icon(ImDrawList* dl, const ImVec2& c, float r)
{
  dl->AddCircle(c, r, WHITE, 32, 1.2f);
  dl->AddLine(add(c, ImVec2(0.0f, -r)), add(c, ImVec2(0.0f, -r * 0.35f)), WHITE, 1.2f);
  dl->AddLine(add(c, ImVec2(0.0f, r)), add(c, ImVec2(0.0f, r * 0.35f)), WHITE, 1.2f);
  dl->AddLine(add(c, ImVec2(-r, 0.0f)), add(c, ImVec2(-r * 0.35f, 0.0f)), WHITE, 1.2f);
  dl->AddLine(add(c, ImVec2(r, 0.0f)), add(c, ImVec2(r * 0.35f, 0.0f)), WHITE, 1.2f);
  const float d = r * 0.30f;
  const ImVec2 diamond[4] = {add(c, ImVec2(0.0f, -d)), add(c, ImVec2(d, 0.0f)), add(c, ImVec2(0.0f, d)),
                             add(c, ImVec2(-d, 0.0f))};
  for (int i = 0; i < 4; i++) {
    dl->AddLine(diamond[i], diamond[(i + 1) % 4], WHITE, 1.2f);
  }
}

// map display: terrain texture seen from above with the aircraft position and heading
inline void draw_map(ImDrawList* dl, const Style& style, const ImVec2& box_min, const ImVec2& box_max,
                     unsigned int texture, const glm::vec3& pos, float heading_deg, float terrain_size,
                     float zoom = 1.0f)
{
  const float margin = 12.0f;
  const ImVec2 map_min = add(box_min, ImVec2(margin, margin));
  const ImVec2 map_max = sub(box_max, ImVec2(margin, margin));

  // aircraft position in uv space: uv = pos.xz / terrain_size + 0.5 (same mapping as the terrain shader)
  const float u_ac = glm::clamp(pos.x / terrain_size + 0.5f, 0.0f, 1.0f);
  const float v_ac = glm::clamp(pos.z / terrain_size + 0.5f, 0.0f, 1.0f);

  // visible uv window: the whole map at zoom 1, a window centered on the aircraft when zoomed in
  const float half = 0.5f / zoom;
  const float u0   = glm::clamp(u_ac - half, 0.0f, 1.0f - 2.0f * half);
  const float v0   = glm::clamp(v_ac - half, 0.0f, 1.0f - 2.0f * half);
  dl->AddImage(reinterpret_cast<ImTextureID>(static_cast<intptr_t>(texture)), map_min, map_max, ImVec2(u0, v0),
               ImVec2(u0 + 2.0f * half, v0 + 2.0f * half));
  dl->AddRect(map_min, map_max, DIM, 0.0f, 0, 1.0f);

  // aircraft position within the visible window
  const float u = (u_ac - u0) / (2.0f * half);
  const float v = (v_ac - v0) / (2.0f * half);
  const ImVec2 ac(map_min.x + u * (map_max.x - map_min.x), map_min.y + v * (map_max.y - map_min.y));

  // aircraft marker pointing along the heading (0 = +x, increasing towards +z = screen down)
  const float a  = glm::radians(heading_deg + 90.0f);
  const ImVec2 d = rot(ImVec2(0.0f, -1.0f), a);  // nose direction
  const ImVec2 s = rot(ImVec2(1.0f, 0.0f), a);   // wing direction
  const ImVec2 tri[3] = {add(ac, mul(d, 9.0f)), add(add(ac, mul(d, -7.0f)), mul(s, 5.0f)),
                         sub(add(ac, mul(d, -7.0f)), mul(s, 5.0f))};
  dl->AddConvexPolyFilled(tri, 3, LIME);
  dl->AddCircle(ac, 10.0f, LIME, 24, 1.2f);
}

// small airplane silhouette, nose along angle
static void draw_plane_icon(ImDrawList* dl, const ImVec2& pos, float size, float angle, ImU32 col)
{
  const ImVec2 d = rot(ImVec2(0.0f, -1.0f), angle);  // nose direction
  const ImVec2 s = rot(ImVec2(1.0f, 0.0f), angle);   // wing direction
  dl->AddLine(sub(pos, mul(d, size)), add(pos, mul(d, size)), col, 1.5f);                       // fuselage
  dl->AddLine(sub(add(pos, mul(d, size * 0.15f)), mul(s, size * 0.8f)),
              add(add(pos, mul(d, size * 0.15f)), mul(s, size * 0.8f)), col, 1.5f);             // wings
  dl->AddLine(sub(sub(pos, mul(d, size * 0.75f)), mul(s, size * 0.35f)),
              add(sub(pos, mul(d, size * 0.75f)), mul(s, size * 0.35f)), col, 1.5f);            // tail
}

// military style radar scope: terrain background centered on the aircraft, numbered degree
// ring, range rings, rotating sweep with a wide phosphor afterglow and airplane targets
inline void draw_radar(ImDrawList* dl, const Style& style, const ImVec2& center, float radius, float time,
                       unsigned int map_texture, const glm::vec3& aircraft_pos, float heading_deg,
                       float terrain_size)
{
  const ImU32 face   = IM_COL32(3, 12, 7, 255);
  const ImU32 grid   = IM_COL32(45, 150, 85, 255);
  const ImU32 bright = IM_COL32(120, 255, 170, 255);

  const float sweep_speed = glm::radians(90.0f);  // one revolution every 4 seconds
  const float sweep       = std::fmod(time * sweep_speed, TWO_PI);

  dl->AddCircleFilled(center, radius, face, 64);

  // terrain background, a +/- 35km window that follows the aircraft, tinted dark green
  const float half_range = 0.35f;  // uv units, terrain_size wide
  const float uc = aircraft_pos.x / terrain_size + 0.5f;
  const float vc = aircraft_pos.z / terrain_size + 0.5f;
  dl->AddImageRounded(reinterpret_cast<ImTextureID>(static_cast<intptr_t>(map_texture)), sub(center, ImVec2(radius, radius)),
                      add(center, ImVec2(radius, radius)), ImVec2(uc - half_range, vc - half_range),
                      ImVec2(uc + half_range, vc + half_range), IM_COL32(0, 200, 100, 140), radius);

  // range rings and crosshair
  for (float f : {0.25f, 0.5f, 0.75f, 1.0f}) {
    dl->AddCircle(center, radius * f, grid, 64, 1.0f);
  }
  dl->AddLine(add(center, ImVec2(-radius, 0.0f)), add(center, ImVec2(radius, 0.0f)), grid, 1.0f);
  dl->AddLine(add(center, ImVec2(0.0f, -radius)), add(center, ImVec2(0.0f, radius)), grid, 1.0f);

  // phosphor afterglow: a wide fan of wedges trailing the sweep with fading alpha
  const int N      = 60;
  const float span = glm::radians(120.0f);
  const float step = span / N;
  const ImVec2 ex(1.0f, 0.0f);
  for (int i = 0; i < N; i++) {
    float a0   = sweep - i * step;
    float a1   = a0 - step;
    float fade = 1.0f - static_cast<float>(i) / N;
    int alpha  = static_cast<int>(150.0f * fade * fade);
    if (alpha <= 0) continue;
    const ImVec2 wedge[3] = {center, add(center, mul(rot(ex, a0), radius)),
                             add(center, mul(rot(ex, a1), radius))};
    dl->AddConvexPolyFilled(wedge, 3, IM_COL32(0, 255, 130, alpha));
  }

  // sweep leading edge
  dl->AddLine(center, add(center, mul(rot(ex, sweep), radius)), bright, 2.0f);
  dl->AddCircleFilled(center, 2.5f, bright, 12);

  // airplane targets at fixed world positions, lit up when the sweep passes over them
  static const struct {
    float x_km, z_km, heading;
  } targets[] = {
      {12.0f, -8.0f, 0.8f},  {-25.0f, 15.0f, 2.2f}, {30.0f, 22.0f, 4.1f}, {-10.0f, -30.0f, 5.3f},
      {5.0f, 33.0f, 1.5f},   {-32.0f, -6.0f, 3.6f}, {18.0f, 3.0f, 2.8f},  {-18.0f, -22.0f, 0.2f},
      {26.0f, -18.0f, 5.9f}, {-5.0f, 24.0f, 4.7f},
  };
  for (const auto& t : targets) {
    // world -> scope position, same mapping as the terrain background
    float sx = (t.x_km * 1000.0f / terrain_size + 0.5f - uc) / half_range;
    float sz = (t.z_km * 1000.0f / terrain_size + 0.5f - vc) / half_range;
    if (sx * sx + sz * sz > 0.95f) continue;  // outside the scope

    ImVec2 pos(center.x + sx * radius, center.y + sz * radius);
    // sweep angle measured like the screen angle: 0 = +x, increasing clockwise
    float ang = std::atan2(sz, sx);
    float rel = std::fmod(sweep - ang + TWO_PI, TWO_PI);  // angle since the sweep passed
    // always dimly visible, flashing bright right after the sweep passes
    int alpha = 70 + static_cast<int>(185.0f * std::exp(-rel * 1.2f));
    draw_plane_icon(dl, pos, 9.0f, t.heading, IM_COL32(120, 255, 170, alpha));
  }

  // own aircraft marker in the center, pointing along the heading
  draw_plane_icon(dl, center, 10.0f, glm::radians(heading_deg + 90.0f), bright);

  // numbered degree ring around the scope
  for (int a = 0; a < 360; a += 5) {
    ImVec2 d   = rot(ImVec2(0.0f, -1.0f), glm::radians(static_cast<float>(a)));
    bool major = (a % 10 == 0);
    dl->AddLine(add(center, mul(d, radius + 3.0f)), add(center, mul(d, radius + (major ? 10.0f : 6.0f))), grid,
                1.0f);
    if (major && a != 0) {
      char buf[4] = {};
      snprintf(buf, sizeof(buf), "%d", a);
      text_centered(dl, style.font, style.font_size - 4.0f, add(center, mul(d, radius + 20.0f)), bright, buf);
    }
  }
}

};  // namespace instruments
