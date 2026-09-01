#pragma once

#include <cmath>
#include <cstdio>

#include <glm/glm.hpp>

#include "../lib/imgui/imgui.h"

// Flight instrument widgets drawn with ImGui draw lists.
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

const ImU32 WHITE  = IM_COL32(235, 240, 245, 255);
const ImU32 YELLOW = IM_COL32(255, 200, 0, 255);
const ImU32 CYAN   = IM_COL32(0, 220, 220, 255);
const ImU32 DIM    = IM_COL32(150, 160, 170, 255);

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

// draw text centered at pos
static inline void text_centered(ImDrawList* dl, ImFont* font, float size, const ImVec2& pos, ImU32 col,
                                 const char* text)
{
  ImVec2 ts = font ? font->CalcTextSizeA(size, FLT_MAX, 0.0f, text) : ImGui::CalcTextSize(text);
  if (font) {
    dl->AddText(font, size, ImVec2(pos.x - ts.x * 0.5f, pos.y - ts.y * 0.5f), col, text);
  } else {
    dl->AddText(ImVec2(pos.x - ts.x * 0.5f, pos.y - ts.y * 0.5f), col, text);
  }
}

// dark dial face with a bezel ring
static void draw_gauge_face(ImDrawList* dl, const ImVec2& c, float r)
{
  dl->AddCircleFilled(c, r + 7.0f, IM_COL32(28, 30, 34, 255), 64);
  dl->AddCircle(c, r + 7.0f, IM_COL32(95, 100, 110, 255), 64, 1.5f);
  dl->AddCircle(c, r + 3.5f, IM_COL32(55, 58, 64, 255), 64, 1.0f);
  dl->AddCircleFilled(c, r, IM_COL32(10, 12, 16, 255), 64);
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
  int npts  = 0;
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
    if (ta > tb) { float tmp = ta; ta = tb; tb = tmp; }
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
  const ImU32 sky_col    = IM_COL32(45, 125, 205, 255);
  const ImU32 ground_col = IM_COL32(130, 85, 40, 255);

  const float px_per_deg = radius / 45.0f;
  const float angle      = -glm::radians(roll_deg);  // the horizon appears to roll opposite to the aircraft
  const float B          = radius * 2.0f;

  const ImVec2 n = rot(ImVec2(0.0f, -1.0f), angle);  // up direction (towards the sky)
  const ImVec2 t = rot(ImVec2(1.0f, 0.0f), angle);   // horizon tangent

  // point on the horizon line, shifted down when pitching up
  const ImVec2 p = add(center, rot(ImVec2(0.0f, pitch_deg * px_per_deg), angle));

  // bezel
  dl->AddCircleFilled(center, radius + 7.0f, IM_COL32(28, 30, 34, 255), 64);
  dl->AddCircle(center, radius + 7.0f, IM_COL32(95, 100, 110, 255), 64, 1.5f);

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
  dl->AddConvexPolyFilled(tri, 3, YELLOW);

  // fixed aircraft symbol
  const float y0 = 0.0f;
  dl->AddLine(add(center, ImVec2(-radius * 0.42f, y0)), add(center, ImVec2(-radius * 0.14f, y0)), YELLOW, 3.5f);
  dl->AddLine(add(center, ImVec2(-radius * 0.14f, y0)), add(center, ImVec2(-radius * 0.14f, y0 + radius * 0.09f)),
              YELLOW, 3.5f);
  dl->AddLine(add(center, ImVec2(radius * 0.14f, y0)), add(center, ImVec2(radius * 0.42f, y0)), YELLOW, 3.5f);
  dl->AddLine(add(center, ImVec2(radius * 0.14f, y0)), add(center, ImVec2(radius * 0.14f, y0 + radius * 0.09f)),
              YELLOW, 3.5f);
  dl->AddCircleFilled(center, 3.0f, YELLOW, 12);
}

// compass card, heading in degrees (0..360, increasing with right turns)
inline void draw_heading_indicator(ImDrawList* dl, const Style& style, const ImVec2& center, float radius,
                                   float heading_deg)
{
  draw_gauge_face(dl, center, radius);

  for (int a = 0; a < 360; a += 10) {
    float rel      = glm::radians(static_cast<float>(a) - heading_deg);
    ImVec2 d       = rot(ImVec2(0.0f, -1.0f), rel);
    bool cardinal  = (a % 90 == 0);
    bool labeled   = (a % 30 == 0);
    float tick_len = cardinal ? 17.0f : (labeled ? 13.0f : 7.0f);
    dl->AddLine(add(center, mul(d, radius - 3.0f)), add(center, mul(d, radius - 3.0f - tick_len)),
                (a == 0) ? IM_COL32(255, 80, 80, 255) : WHITE, labeled ? 2.0f : 1.0f);

    if (labeled) {
      char buf[4] = {};
      if (cardinal) {
        const char* letters = "NESW";
        snprintf(buf, sizeof(buf), "%c", letters[a / 90]);
      } else {
        snprintf(buf, sizeof(buf), "%d", a / 10);
      }
      text_centered(dl, style.font, style.font_size, add(center, mul(d, radius - 32.0f)),
                    (a == 0) ? IM_COL32(255, 80, 80, 255) : WHITE, buf);
    }
  }

  // fixed lubber line at the top
  const ImVec2 tri[3] = {add(center, ImVec2(-6.0f, -radius + 2.0f)), add(center, ImVec2(6.0f, -radius + 2.0f)),
                         add(center, ImVec2(0.0f, -radius + 15.0f))};
  dl->AddConvexPolyFilled(tri, 3, YELLOW);

  // fixed aircraft symbol in the middle
  dl->AddCircleFilled(center, 3.0f, YELLOW, 12);
  dl->AddLine(add(center, ImVec2(-radius * 0.22f, 0.0f)), add(center, ImVec2(radius * 0.22f, 0.0f)), YELLOW, 2.5f);
  dl->AddLine(add(center, ImVec2(0.0f, -radius * 0.22f)), add(center, ImVec2(0.0f, radius * 0.16f)), YELLOW, 2.5f);

  // digital heading readout at the bottom
  char buf[8] = {};
  int hdg     = ((static_cast<int>(std::lround(heading_deg)) % 360) + 360) % 360;
  snprintf(buf, sizeof(buf), "%03d", hdg);
  ImVec2 ts = style.font_big ? style.font_big->CalcTextSizeA(style.font_big_size, FLT_MAX, 0.0f, buf)
                             : ImGui::CalcTextSize(buf);
  ImVec2 bc(center.x, center.y + radius * 0.52f);
  ImVec2 box_min(bc.x - ts.x * 0.5f - 8.0f, bc.y - ts.y * 0.5f - 4.0f);
  ImVec2 box_max(bc.x + ts.x * 0.5f + 8.0f, bc.y + ts.y * 0.5f + 4.0f);
  dl->AddRectFilled(box_min, box_max, IM_COL32(0, 0, 0, 220), 4.0f);
  dl->AddRect(box_min, box_max, DIM, 4.0f, 0, 1.0f);
  if (style.font_big) {
    dl->AddText(style.font_big, style.font_big_size, ImVec2(bc.x - ts.x * 0.5f, bc.y - ts.y * 0.5f), WHITE, buf);
  } else {
    dl->AddText(ImVec2(bc.x - ts.x * 0.5f, bc.y - ts.y * 0.5f), WHITE, buf);
  }
}

// vertical moving tape (for speed, altitude, ...), value is centered and the scale moves
inline void draw_tape(ImDrawList* dl, const Style& style, const ImVec2& pos, const ImVec2& size, const char* title,
                      float value, float px_per_unit, float minor_step, int label_every, bool ticks_on_left)
{
  const ImVec2 vmin = pos;
  const ImVec2 vmax = add(pos, size);
  const float cy    = pos.y + size.y * 0.5f;
  const float tick_edge = ticks_on_left ? vmin.x : vmax.x;
  const float tick_dir  = ticks_on_left ? 1.0f : -1.0f;

  text_centered(dl, style.font, style.font_size, ImVec2(pos.x + size.x * 0.5f, pos.y - 12.0f), DIM, title);

  dl->AddRectFilled(vmin, vmax, IM_COL32(10, 12, 16, 235));
  dl->PushClipRect(vmin, vmax, true);

  const float half_range = size.y * 0.5f / px_per_unit;
  const float first      = std::floor((value - half_range) / minor_step) * minor_step;
  const int count        = static_cast<int>(std::ceil(2.0f * half_range / minor_step)) + 1;

  for (int i = 0; i <= count; i++) {
    float v = first + static_cast<float>(i) * minor_step;
    if (v < 0.0f) continue;

    float y    = cy - (v - value) * px_per_unit;
    bool major = (i % label_every == 0);
    float tl   = major ? 13.0f : 7.0f;
    dl->AddLine(ImVec2(tick_edge, y), ImVec2(tick_edge + tick_dir * tl, y), WHITE, major ? 2.0f : 1.0f);

    if (major) {
      char buf[16] = {};
      snprintf(buf, sizeof(buf), "%d", static_cast<int>(std::lround(v)));
      float tx = ticks_on_left ? vmin.x + tl + 5.0f : vmax.x - tl - 5.0f;
      ImVec2 ts = style.font ? style.font->CalcTextSizeA(style.font_size, FLT_MAX, 0.0f, buf)
                             : ImGui::CalcTextSize(buf);
      if (!ticks_on_left) tx -= ts.x;
      if (style.font) {
        dl->AddText(style.font, style.font_size, ImVec2(tx, y - ts.y * 0.5f), WHITE, buf);
      } else {
        dl->AddText(ImVec2(tx, y - ts.y * 0.5f), WHITE, buf);
      }
    }
  }

  dl->PopClipRect();
  dl->AddRect(vmin, vmax, DIM, 0.0f, 0, 1.0f);

  // digital readout box with a pointer towards the scale
  char buf[16] = {};
  snprintf(buf, sizeof(buf), "%d", static_cast<int>(std::lround(value)));
  ImVec2 ts = style.font_big ? style.font_big->CalcTextSizeA(style.font_big_size, FLT_MAX, 0.0f, buf)
                             : ImGui::CalcTextSize(buf);
  const float pad = 5.0f;
  ImVec2 box_min(vmin.x - pad, cy - ts.y * 0.5f - 4.0f);
  ImVec2 box_max(vmax.x + pad, cy + ts.y * 0.5f + 4.0f);
  dl->AddRectFilled(box_min, box_max, IM_COL32(0, 0, 0, 240), 3.0f);
  dl->AddRect(box_min, box_max, WHITE, 3.0f, 0, 1.5f);

  // pointer triangle sticking out of the box towards the tape edge
  float px          = ticks_on_left ? box_min.x : box_max.x;
  float pdir        = ticks_on_left ? -7.0f : 7.0f;
  const ImVec2 tri[3] = {ImVec2(px + pdir, cy), ImVec2(px, cy - 6.0f), ImVec2(px, cy + 6.0f)};
  dl->AddConvexPolyFilled(tri, 3, WHITE);

  if (style.font_big) {
    dl->AddText(style.font_big, style.font_big_size, ImVec2(pos.x + size.x * 0.5f - ts.x * 0.5f, cy - ts.y * 0.5f),
                YELLOW, buf);
  } else {
    dl->AddText(ImVec2(pos.x + size.x * 0.5f - ts.x * 0.5f, cy - ts.y * 0.5f), YELLOW, buf);
  }
}

};  // namespace instruments
