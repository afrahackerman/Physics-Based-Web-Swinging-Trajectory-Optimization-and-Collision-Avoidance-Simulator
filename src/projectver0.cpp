// MinGW workaround
// g++ project.cpp -o project.exe -lbgi -lgdi32 -lcomdlg32 -luuid -loleaut32 -lole32

#include <graphics.h>
#include <windows.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <vector>
/*
$env:Path = "C:\msys64\ucrt64\bin;" + $env:Path
& "C:\msys64\ucrt64\bin\g++.exe" projectver0.cpp -o projectver0.exe -std=c++17 -I"C:\Users\afra\Downloads\WinBGIm-64-1.0.1\WinBGIm-64-1.0.1\libbgi\include\bgi" -L"C:\Users\afra\Downloads\WinBGIm-64-1.0.1\WinBGIm-64-1.0.1\build\libbgi" -lbgi -lgdi32 -lcomdlg32 -luuid -loleaut32 -lole32
$LASTEXITCODE
.\projectver0.exe
*/

// ===== STAGE 1/3: constants, world->screen transformation (proposal Eq.1-3) =====
const int SCREEN_W = 1200, SCREEN_H = 600;
const double X_MIN = 0.0, X_MAX = 120.0, Y_MIN = 0.0, Y_MAX = 70.0;
const double SX = (SCREEN_W - 1) / (X_MAX - X_MIN);
const double SY = (SCREEN_H - 1) / (Y_MAX - Y_MIN);
const double GROUND_Y = 5.0;

int toScreenX(double wx) { return (int)floor(SX * (wx - X_MIN) + 0.5); }
int toScreenY(double wy) { return (SCREEN_H - 1) - (int)floor(SY * (wy - Y_MIN) + 0.5); }

// ---------- physics / search constants ----------
const double G = 9.81;
const double PI = 3.14159265358979323846;
const double DEG2RAD = PI / 180.0, RAD2DEG = 180.0 / PI;
const double DT = 1.0 / 120.0;       // animation step
const double SEARCH_DT = 1.0 / 60.0; // search step (set = DT for exact match)
const double BODY_R = 0.8;
const double LMIN = 8.0, LMAX = 40.0;
const double GOAL_EPS = 5.0;
const int MAX_SWINGS = 3;
const double SWING_MAX_T = 6.0, FLIGHT_MAX_T = 6.0;

const int NUM_ANGLES = 6;
const double RELEASE_ANGLES[NUM_ANGLES] = {-30, -10, 10, 30, 50, 70};
const int NUM_ATTACH = 3;
const double ATTACH_DIST[NUM_ATTACH] = {16, 22, 28};

// ---------- data structures (world coordinates, metres) ----------
struct Building
{
  double x, y, w, h;
}; // (x,y) bottom-left
struct Anchor
{
  double x, y;
};
struct Point2
{
  double x, y;

  Point2(double px = 0.0, double py = 0.0)
      : x(px), y(py) {}
};
struct Body
{
  double x, y, vx, vy;
};
struct Swing
{
  double ax, ay, L, theta, omega;
};
struct RouteStep
{
  int anchor;
  double angleDeg;
};

const int NUM_BUILDINGS = 5, NUM_ANCHORS = 4;
Building buildings[NUM_BUILDINGS];
Anchor anchors[NUM_ANCHORS] = {{26, 60}, {52, 60}, {78, 60}, {100, 60}};
Point2 startPoint(8, 56);
Point2 goalPoint(108, 25); // roof of last building
int currentLevel = 1;

void loadLevel(int n)
{
  Building base[NUM_BUILDINGS] = {
      {14, GROUND_Y, 10, 18}, {34, GROUND_Y, 8, 22}, {54, GROUND_Y, 10, 18}, {76, GROUND_Y, 9, 24}, {98, GROUND_Y, 18, 20}};
  for (int i = 0; i < NUM_BUILDINGS; i++)
    buildings[i] = base[i];
  if (n == 2)
  {
    buildings[2].x = 57;
    buildings[2].w = 6;
    buildings[2].h = 65;
  } // full-height wall: no safe route
}

// ===== STAGE 6: collision detection =====
double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

bool circleHitsRect(double cx, double cy, double r, const Building &b)
{
  double nx = clampd(cx, b.x, b.x + b.w), ny = clampd(cy, b.y, b.y + b.h);
  double dx = cx - nx, dy = cy - ny;
  return dx * dx + dy * dy < r * r;
}

// Liang-Barsky / slab test: does segment (x0,y0)-(x1,y1) intersect the rectangle?
bool segmentHitsRect(double x0, double y0, double x1, double y1, const Building &b)
{
  double t0 = 0.0, t1 = 1.0, dx = x1 - x0, dy = y1 - y0;
  double p[4] = {-dx, dx, -dy, dy};
  double q[4] = {x0 - b.x, (b.x + b.w) - x0, y0 - b.y, (b.y + b.h) - y0};
  for (int i = 0; i < 4; i++)
  {
    if (fabs(p[i]) < 1e-12)
    {
      if (q[i] < 0)
        return false;
    }
    else
    {
      double t = q[i] / p[i];
      if (p[i] < 0)
      {
        if (t > t1)
          return false;
        if (t > t0)
          t0 = t;
      }
      else
      {
        if (t < t0)
          return false;
        if (t < t1)
          t1 = t;
      }
    }
  }
  return true;
}

bool ropeClear(int ai, const Body &b)
{
  for (int i = 0; i < NUM_BUILDINGS; i++)
    if (segmentHitsRect(anchors[ai].x, anchors[ai].y, b.x, b.y, buildings[i]))
      return false;
  return true;
}

bool outOfWorld(const Body &b)
{
  return b.x < X_MIN + BODY_R || b.x > X_MAX - BODY_R || b.y > Y_MAX - BODY_R;
}

// While swinging, ANY contact (body, rope, ground, edge) is a crash
bool swingCrash(const Body &b, int ai)
{
  if (outOfWorld(b) || b.y - BODY_R <= GROUND_Y)
    return true;
  for (int i = 0; i < NUM_BUILDINGS; i++)
    if (circleHitsRect(b.x, b.y, BODY_R, buildings[i]) ||
        segmentHitsRect(anchors[ai].x, anchors[ai].y, b.x, b.y, buildings[i]))
      return true;
  return false;
}

enum Contact
{
  C_NONE,
  C_LAND,
  C_CRASH
};
// In flight: touching a roof/ground from above = landing, anything else = crash
Contact flightContact(Body &b)
{
  if (outOfWorld(b))
    return C_CRASH;
  for (int i = 0; i < NUM_BUILDINGS; i++)
  {
    const Building &bd = buildings[i];
    if (circleHitsRect(b.x, b.y, BODY_R, bd))
    {
      double top = bd.y + bd.h;
      if (b.y >= top && b.vy <= 0)
      {
        double dx = b.x - clampd(b.x, bd.x, bd.x + bd.w);
        double h2 = BODY_R * BODY_R - dx * dx;
        b.y = top + sqrt(h2 > 0 ? h2 : 0);
        b.vx = b.vy = 0;
        return C_LAND;
      }
      return C_CRASH;
    }
  }
  if (b.y - BODY_R <= GROUND_Y)
  {
    b.y = GROUND_Y + BODY_R;
    b.vx = b.vy = 0;
    return C_LAND;
  }
  return C_NONE;
}

bool atGoal(const Body &b)
{
  return fabs(b.x - goalPoint.x) <= GOAL_EPS && fabs(b.y - (goalPoint.y + BODY_R)) <= 0.5;
}

// ===== STAGE 4: pendulum (semi-implicit Euler) =====
void swingStep(Swing &s, double dt)
{
  s.omega -= (G / s.L) * sin(s.theta) * dt; // Eq.7
  s.theta += s.omega * dt;                  // Eq.8 (uses the NEW omega)
}
void swingPos(const Swing &s, Body &b) // Eq.5, Eq.6
{
  b.x = s.ax + s.L * sin(s.theta);
  b.y = s.ay - s.L * cos(s.theta);
  b.vx = s.L * s.omega * cos(s.theta);
  b.vy = s.L * s.omega * sin(s.theta);
}
// attach the web: rope length = current distance, keep only the tangential speed
bool makeSwing(int ai, const Body &b, Swing &s)
{
  double dx = b.x - anchors[ai].x, dy = anchors[ai].y - b.y;
  double L = sqrt(dx * dx + dy * dy);
  if (L < LMIN || L > LMAX)
    return false;
  if (dy < 0.1 * L)
    return false; // body must hang below the anchor (rope taut)
  s.ax = anchors[ai].x;
  s.ay = anchors[ai].y;
  s.L = L;
  s.theta = atan2(dx, dy);
  s.omega = (b.vx * cos(s.theta) + b.vy * sin(s.theta)) / L;
  return true;
}
double swingEnergy(const Swing &s) { return 0.5 * s.L * s.L * s.omega * s.omega - G * s.L * cos(s.theta); } // Eq.9

// ===== STAGE 5: projectile (same semi-implicit scheme) =====
void flightStep(Body &b, double dt)
{
  b.vy -= G * dt;
  b.x += b.vx * dt;
  b.y += b.vy * dt;
}
double predictLandingX(const Body &b, double yL) // Eq.12
{
  double disc = b.vy * b.vy + 2 * G * (b.y - yL);
  if (disc < 0)
    return b.x;
  return b.x + b.vx * (b.vy + sqrt(disc)) / G;
}

// ===== STAGE 7: route search (depth-first, <=3 swings, branch pruning) =====
double bestTime = 1e18;
bool hasBest = false;
int bestCount = 0;
long nodes = 0;
RouteStep curSteps[MAX_SWINGS], bestSteps[MAX_SWINGS];
std::vector<Point2> trace, bestTrace;
char routeText[128] = "";

void searchFlight(Body b, double t0, int depth, int fromAnchor);

void searchSwing(int ai, const Body &start, double t0, int depth)
{
  if (t0 >= bestTime)
    return;
  Swing s;
  if (!makeSwing(ai, start, s) || !ropeClear(ai, start))
    return;
  size_t mark = trace.size();
  double t = t0, prevTheta = s.theta;
  Body b = start;
  while (t - t0 < SWING_MAX_T && t < bestTime)
  {
    swingStep(s, SEARCH_DT);
    t += SEARCH_DT;
    swingPos(s, b);
    if (swingCrash(b, ai))
      break; // reject candidate
    trace.push_back(Point2(b.x, b.y));
    if (b.vx > 0)
    {
      for (int k = 0; k < NUM_ANGLES; k++)
      {
        double th = RELEASE_ANGLES[k] * DEG2RAD;
        if ((prevTheta < th && s.theta >= th) || (prevTheta > th && s.theta <= th))
        {
          curSteps[depth].anchor = ai;
          curSteps[depth].angleDeg = RELEASE_ANGLES[k];
          size_t m2 = trace.size();
          searchFlight(b, t, depth + 1, ai);
          trace.resize(m2);
        }
      }
    }
    prevTheta = s.theta;
  }
  trace.resize(mark);
}

void searchFlight(Body b, double t0, int depth, int fromAnchor)
{
  if (t0 >= bestTime)
    return;
  nodes++;
  // pruning: even at the maximum possible speed (energy bound) the goal cannot be reached sooner
  double hDrop = b.y - (GROUND_Y + BODY_R);
  if (hDrop < 0)
    hDrop = 0;
  double vmax = sqrt(b.vx * b.vx + b.vy * b.vy + 2.0 * G * hDrop);
  double dxg = fabs(goalPoint.x - b.x) - GOAL_EPS;
  if (dxg < 0)
    dxg = 0;
  if (vmax > 1e-9 && t0 + dxg / vmax >= bestTime)
    return;

  bool done[NUM_ANCHORS][NUM_ATTACH] = {};
  size_t mark = trace.size();
  double t = t0;
  while (t - t0 < FLIGHT_MAX_T && t < bestTime)
  {
    if (depth < MAX_SWINGS)
    { // try attaching to every other anchor
      for (int j = 0; j < NUM_ANCHORS; j++)
      {
        if (j == fromAnchor)
          continue;
        double dx = b.x - anchors[j].x, dy = b.y - anchors[j].y, d2 = dx * dx + dy * dy;
        bool fire = false;
        for (int m = 0; m < NUM_ATTACH; m++)
          if (!done[j][m] && d2 <= ATTACH_DIST[m] * ATTACH_DIST[m])
          {
            done[j][m] = true;
            fire = true;
          }
        if (fire)
        {
          size_t m2 = trace.size();
          searchSwing(j, b, t, depth);
          trace.resize(m2);
        }
      }
    }
    flightStep(b, SEARCH_DT);
    t += SEARCH_DT;
    Contact c = flightContact(b);
    trace.push_back(Point2(b.x, b.y));
    if (c == C_NONE)
      continue;
    if (c == C_LAND && atGoal(b) && t < bestTime)
    { // valid route, faster than best
      bestTime = t;
      bestCount = depth;
      hasBest = true;
      for (int i = 0; i < depth; i++)
        bestSteps[i] = curSteps[i];
      bestTrace = trace;
    }
    break; // landed elsewhere or crashed
  }
  trace.resize(mark);
}

bool findBestRoute()
{
  bestTime = 1e18;
  hasBest = false;
  bestCount = 0;
  nodes = 0;
  trace.clear();
  bestTrace.clear();
  routeText[0] = 0;
  Body s0 = {startPoint.x, startPoint.y, 0, 0};
  trace.push_back(startPoint);
  for (int a = 0; a < NUM_ANCHORS; a++)
  {
    size_t m = trace.size();
    searchSwing(a, s0, 0.0, 0);
    trace.resize(m);
  }
  if (hasBest)
  {
    char tmp[32];
    for (int i = 0; i < bestCount; i++)
    {
      sprintf(tmp, "A%d@%d%s", bestSteps[i].anchor + 1, (int)bestSteps[i].angleDeg, i + 1 < bestCount ? " > " : "");
      strcat(routeText, tmp);
    }
  }
  return hasBest;
}

// ===== STAGE 2/4/5/8: manual simulation state =====
enum Mode
{
  M_STAND,
  M_SWING,
  M_FLIGHT,
  M_CRASHED
};
Mode mode = M_STAND;
Body body;
Swing swing;
int selAnchor = 0, attachedAnchor = -1;
double releaseAngleDeg = 30.0, elapsed = 0.0, predLandX = 0.0;
bool goalReached = false, hasPred = false;
char status[64], collisionMsg[32], searchMsg[96];
std::vector<Point2> trail;
int trailCounter = 0;
bool keyDown[256], keyPressed[256];

void resetAll()
{
  body.x = startPoint.x;
  body.y = startPoint.y;
  body.vx = body.vy = 0;
  mode = M_STAND;
  attachedAnchor = -1;
  elapsed = 0;
  goalReached = false;
  hasPred = false;
  trail.clear();
  bestTrace.clear();
  hasBest = false;
  bestTime = 1e18;
  routeText[0] = 0;
  strcpy(status, "READY - SPACE to attach");
  strcpy(collisionMsg, "NONE");
  strcpy(searchMsg, "Search: not run (press ENTER)");
}

void tryAttach()
{
  if (!makeSwing(selAnchor, body, swing))
  {
    strcpy(status, "Anchor out of range / not above");
    return;
  }
  if (!ropeClear(selAnchor, body))
  {
    strcpy(status, "Rope blocked by building");
    return;
  }
  attachedAnchor = selAnchor;
  mode = M_SWING;
  strcpy(status, "Swinging");
}

void releaseWeb()
{
  mode = M_FLIGHT;
  attachedAnchor = -1;
  predLandX = predictLandingX(body, GROUND_Y + BODY_R);
  hasPred = true;
  strcpy(status, "Web released - flying");
}

void stepPhysics()
{
  if (mode == M_SWING)
  {
    double prev = swing.theta;
    swingStep(swing, DT);
    swingPos(swing, body);
    elapsed += DT;
    if (swingCrash(body, attachedAnchor))
    {
      mode = M_CRASHED;
      strcpy(collisionMsg, "CRASH (swing/rope)");
      strcpy(status, "Crashed - press R");
      return;
    }
    double th = releaseAngleDeg * DEG2RAD;
    bool crossed = (prev < th && swing.theta >= th) || (prev > th && swing.theta <= th);
    if (crossed && body.vx > 0)
      releaseWeb(); // auto-release at preset angle
  }
  else if (mode == M_FLIGHT)
  {
    flightStep(body, DT);
    elapsed += DT;
    Contact c = flightContact(body);
    if (c == C_CRASH)
    {
      mode = M_CRASHED;
      strcpy(collisionMsg, "CRASH (building/edge)");
      strcpy(status, "Crashed - press R");
    }
    else if (c == C_LAND)
    {
      mode = M_STAND;
      if (atGoal(body))
      {
        goalReached = true;
        strcpy(status, "GOAL REACHED!");
      }
      else
        strcpy(status, "Landed (not at goal)");
    }
  }
  if ((mode == M_SWING || mode == M_FLIGHT) &&
      ++trailCounter % 4 == 0)
  {
    trail.push_back(Point2(body.x, body.y));
  }
}

void updateKeys()
{
  for (int i = 0; i < 256; i++)
  {
    bool d = (GetAsyncKeyState(i) & 0x8000) != 0;
    keyPressed[i] = d && !keyDown[i];
    keyDown[i] = d;
  }
}

// ===== STAGE 1/2/8: drawing =====
void drawSky()
{
  int groundTop = toScreenY(GROUND_Y);
  for (int sy = 0; sy < groundTop; sy += 5)
  {
    double t = (double)sy / groundTop;
    setfillstyle(SOLID_FILL, COLOR((int)(40 + 130 * t), (int)(80 + 130 * t), (int)(160 + 95 * t)));
    bar(0, sy, SCREEN_W, sy + 5);
  }
}
void drawGround()
{
  int top = toScreenY(GROUND_Y);
  setfillstyle(SOLID_FILL, COLOR(60, 60, 60));
  bar(0, top, SCREEN_W, SCREEN_H);
  setcolor(WHITE);
  line(0, top, SCREEN_W, top);
}
void drawBuildings()
{
  for (int i = 0; i < NUM_BUILDINGS; i++)
  {
    int l = toScreenX(buildings[i].x), r = toScreenX(buildings[i].x + buildings[i].w);
    int t = toScreenY(buildings[i].y + buildings[i].h), b = toScreenY(buildings[i].y);
    setfillstyle(SOLID_FILL, COLOR(90, 90, 110));
    bar(l, t, r, b);
    setcolor(BLACK);
    rectangle(l, t, r, b);
  }
}
void drawAnchors()
{
  char label[8];
  for (int i = 0; i < NUM_ANCHORS; i++)
  {
    int sx = toScreenX(anchors[i].x), sy = toScreenY(anchors[i].y);
    setcolor(YELLOW);
    setfillstyle(SOLID_FILL, YELLOW);
    fillellipse(sx, sy, 5, 5);
    if (i == selAnchor)
    {
      setcolor(i == attachedAnchor ? GREEN : WHITE);
      circle(sx, sy, 10);
    }
    sprintf(label, "A%d", i + 1);
    setcolor(WHITE);
    outtextxy(sx - 8, sy - 24, label);
  }
}
void drawStartGoal()
{
  int sx = toScreenX(startPoint.x), sy = toScreenY(startPoint.y);
  setcolor(GREEN);
  circle(sx, sy, 14);
  outtextxy(sx - 15, sy + 20, (char *)"START");
  int gy = toScreenY(goalPoint.y);
  setcolor(RED);
  setlinestyle(SOLID_LINE, 0, 3);
  line(toScreenX(goalPoint.x - GOAL_EPS), gy, toScreenX(goalPoint.x + GOAL_EPS), gy);
  setlinestyle(SOLID_LINE, 0, 1);
  outtextxy(toScreenX(goalPoint.x) - 12, gy - 18, (char *)"GOAL");
}
void drawCharacter()
{
  int cx = toScreenX(body.x), cy = toScreenY(body.y);
  setcolor(BLUE);
  line(cx, cy + 6, cx - 7, cy + 16);
  line(cx, cy + 6, cx + 7, cy + 16); // legs
  line(cx - 8, cy, cx - 14, cy + 7);
  line(cx + 8, cy, cx + 14, cy + 7); // arms
  setcolor(BLACK);
  setfillstyle(SOLID_FILL, RED);
  fillellipse(cx, cy, 8, 8);
  line(cx - 8, cy, cx + 8, cy);
  line(cx, cy - 8, cx, cy + 8); // web pattern
  setfillstyle(SOLID_FILL, WHITE);
  fillellipse(cx - 3, cy - 3, 2, 3);
  fillellipse(cx + 3, cy - 3, 2, 3); // eyes
}
void drawRope()
{
  if (mode != M_SWING)
    return;
  setcolor(WHITE);
  line(toScreenX(anchors[attachedAnchor].x), toScreenY(anchors[attachedAnchor].y), toScreenX(body.x), toScreenY(body.y));
}
void drawGhost()
{
  if (bestTrace.size() < 2)
    return;
  setcolor(CYAN);
  setlinestyle(DOTTED_LINE, 0, 1);
  for (size_t i = 1; i < bestTrace.size(); i++)
    line(toScreenX(bestTrace[i - 1].x), toScreenY(bestTrace[i - 1].y), toScreenX(bestTrace[i].x), toScreenY(bestTrace[i].y));
  setlinestyle(SOLID_LINE, 0, 1);
}
void drawTrail()
{
  setcolor(COLOR(255, 140, 0));
  for (size_t i = 1; i < trail.size(); i++)
    line(toScreenX(trail[i - 1].x), toScreenY(trail[i - 1].y), toScreenX(trail[i].x), toScreenY(trail[i].y));
  if (hasPred)
  {
    int px = toScreenX(predLandX), py = toScreenY(GROUND_Y);
    setcolor(RED);
    line(px - 5, py - 5, px + 5, py + 5);
    line(px - 5, py + 5, px + 5, py - 5);
  }
}
void drawHUD()
{
  char buf[128];
  int x = 12, y = 10;
  const char *names[] = {"STAND", "SWING", "FLIGHT", "CRASHED"};
  setfillstyle(SOLID_FILL, COLOR(15, 15, 25));
  bar(5, 5, 400, 208);
  setcolor(WHITE);
  rectangle(5, 5, 400, 208);
  sprintf(buf, "C233456 | Level %d | Mode: %s", currentLevel, names[mode]);
  outtextxy(x, y, buf);
  y += 17;
  double speed = sqrt(body.vx * body.vx + body.vy * body.vy);
  sprintf(buf, "Time: %.2f s   Speed: %.1f m/s", elapsed, speed);
  outtextxy(x, y, buf);
  y += 17;
  if (mode == M_SWING)
    sprintf(buf, "Swing angle: %.1f deg   Release: %.0f deg", swing.theta * RAD2DEG, releaseAngleDeg);
  else
    sprintf(buf, "Swing angle: --   Release: %.0f deg", releaseAngleDeg);
  outtextxy(x, y, buf);
  y += 17;
  if (mode == M_SWING)
    sprintf(buf, "Energy/mass: %.2f J/kg", swingEnergy(swing));
  else
    sprintf(buf, "Energy/mass: --");
  outtextxy(x, y, buf);
  y += 17;
  if (attachedAnchor >= 0)
    sprintf(buf, "Anchor: selected A%d, attached A%d", selAnchor + 1, attachedAnchor + 1);
  else
    sprintf(buf, "Anchor: selected A%d, attached none", selAnchor + 1);
  outtextxy(x, y, buf);
  y += 17;
  sprintf(buf, "Collision: %s", collisionMsg);
  outtextxy(x, y, buf);
  y += 17;
  sprintf(buf, "Status: %s", status);
  outtextxy(x, y, buf);
  y += 17;
  if (hasBest)
    sprintf(buf, "Best route time: %.2f s", bestTime);
  else
    sprintf(buf, "Best route time: --");
  outtextxy(x, y, buf);
  y += 17;
  sprintf(buf, "Route: %s", hasBest ? routeText : "--");
  outtextxy(x, y, buf);
  y += 17;
  outtextxy(x, y, searchMsg);
  y += 17;
  if (hasPred)
    sprintf(buf, "Pred. ground landing x: %.1f m", predLandX);
  else
    sprintf(buf, "Pred. ground landing x: --");
  outtextxy(x, y, buf);
}
void drawHelp()
{
  int y = toScreenY(GROUND_Y) + 12;
  setcolor(WHITE);
  outtextxy(10, y, (char *)"SPACE attach/release | LEFT/RIGHT anchor | UP/DOWN release angle | ENTER search | R reset");
  outtextxy(10, y + 18, (char *)"L switch level (1 open / 2 blocked) | T self-test (console) | ESC exit");
}
void renderFrame(int &page)
{
  setactivepage(page);
  drawSky();
  drawGround();
  drawBuildings();
  drawStartGoal();
  drawAnchors();
  drawGhost();
  drawTrail();
  drawRope();
  drawCharacter();
  drawHUD();
  drawHelp();
  setvisualpage(page);
  page = 1 - page;
}

// ===== STAGE 9: self-tests (printed in the console window) =====
void check(const char *name, bool ok) { printf("%-46s %s\n", name, ok ? "PASS" : "FAIL"); }
void selfTest()
{
  printf("\n===== SELF TEST =====\n");
  check("map (0,0)   -> (0,699)", toScreenX(0) == 0 && toScreenY(0) == SCREEN_H - 1);
  check("map (120,70)-> (1199,0)", toScreenX(120) == SCREEN_W - 1 && toScreenY(70) == 0);

  double L = 20.0, Tth = 2 * PI * sqrt(L / G); // pendulum period
  Swing s = {0, 0, L, 5 * DEG2RAD, 0};
  double t = 0, prev = s.theta;
  while (s.theta > 0 && t < 20)
  {
    prev = s.theta;
    swingStep(s, DT);
    t += DT;
  }
  double tq = (t - DT) + DT * prev / (prev - s.theta);
  double Tsim = 4 * tq;
  printf("period: sim %.4f s, theory %.4f s, error %.3f %%\n", Tsim, Tth, 100 * fabs(Tsim - Tth) / Tth);
  check("period error < 1 %", fabs(Tsim - Tth) / Tth < 0.01);

  Swing e = {0, 0, L, 60 * DEG2RAD, 0}; // energy over ~10 periods
  double E0 = -G * L * cos(e.theta), maxErr = 0;
  int n = (int)(10 * Tth / DT);
  for (int i = 0; i < n; i++)
  {
    swingStep(e, DT);
    double err = fabs(swingEnergy(e) - E0) / fabs(E0);
    if (err > maxErr)
      maxErr = err;
  }
  printf("energy: max relative change %.4f %%\n", 100 * maxErr);
  check("energy change < 1 %", maxErr < 0.01);

  Body p = {0, 40, 10, 5}; // projectile to y=0
  double tf = (p.vy + sqrt(p.vy * p.vy + 2 * G * p.y)) / G, xTh = p.vx * tf;
  double px = 0, py = 0;
  while (p.y > 0)
  {
    px = p.x;
    py = p.y;
    flightStep(p, DT);
  }
  double xSim = px + (py / (py - p.y)) * (p.x - px);
  printf("projectile: sim x=%.3f m, theory x=%.3f m, diff %.2f px\n", xSim, xTh, fabs(xSim - xTh) * SX);
  check("landing within 1 pixel", fabs(xSim - xTh) * SX < 1.0);

  Building tb = {10, 0, 10, 10}; // collisions
  check("circle (5,5) r=4.9 misses box", !circleHitsRect(5, 5, 4.9, tb));
  check("circle (5,5) r=5.1 hits box", circleHitsRect(5, 5, 5.1, tb));
  check("segment above box misses", !segmentHitsRect(0, 10.5, 30, 10.5, tb));
  check("segment through box hits", segmentHitsRect(0, 5, 30, 5, tb));
  check("vertical segment through box hits", segmentHitsRect(15, 20, 15, 5, tb));
  printf("===== END =====\n\n");
}

// ===== STAGE 2/9: main loop =====
int main()
{
  initwindow(SCREEN_W, SCREEN_H, (char *)"Web-Swinging Simulator");
  loadLevel(currentLevel);
  resetAll();
  int page = 0;
  clock_t last = clock();
  double acc = 0.0;
  bool running = true;
  while (running)
  {
    clock_t now = clock();
    double ft = (double)(now - last) / CLOCKS_PER_SEC;
    last = now;
    if (ft > 0.1)
      ft = 0.1;
    acc += ft;

    updateKeys();
    if (keyPressed[VK_ESCAPE])
      break;
    if (keyPressed['R'])
      resetAll();
    if (keyPressed['L'])
    {
      currentLevel = 3 - currentLevel;
      loadLevel(currentLevel);
      resetAll();
    }
    if (keyPressed['T'])
      selfTest();
    if (keyPressed[VK_LEFT])
      selAnchor = (selAnchor + NUM_ANCHORS - 1) % NUM_ANCHORS;
    if (keyPressed[VK_RIGHT])
      selAnchor = (selAnchor + 1) % NUM_ANCHORS;
    if (keyPressed[VK_UP])
    {
      releaseAngleDeg += 5;
      if (releaseAngleDeg > 80)
        releaseAngleDeg = 80;
    }
    if (keyPressed[VK_DOWN])
    {
      releaseAngleDeg -= 5;
      if (releaseAngleDeg < -30)
        releaseAngleDeg = -30;
    }
    if (keyPressed[VK_SPACE])
    {
      if (mode == M_SWING)
        releaseWeb();
      else if (mode == M_STAND || mode == M_FLIGHT)
        tryAttach();
    }
    if (keyPressed[VK_RETURN])
    {
      strcpy(searchMsg, "Search: RUNNING...");
      renderFrame(page); // show the message before blocking
      clock_t c0 = clock();
      bool ok = findBestRoute();
      double secs = (double)(clock() - c0) / CLOCKS_PER_SEC;
      if (ok)
      {
        sprintf(searchMsg, "Search: done %.2fs, %ld flights tested", secs, nodes);
        selAnchor = bestSteps[0].anchor;
        releaseAngleDeg = bestSteps[0].angleDeg;
      }
      else
        sprintf(searchMsg, "Search: NO VALID ROUTE (%.2fs)", secs);
      last = clock();
      acc = 0;
    }

    while (acc >= DT)
    {
      stepPhysics();
      acc -= DT;
    } // fixed time step
    renderFrame(page);
    delay(8);
  }
  closegraph();
  return 0;
}