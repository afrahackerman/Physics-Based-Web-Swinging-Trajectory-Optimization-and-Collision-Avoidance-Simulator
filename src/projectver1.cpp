// MinGW workaround
// g++ project.cpp -o project.exe -std=c++17 -lbgi -lgdi32 -lcomdlg32 -luuid -loleaut32 -lole32

#include <graphics.h>
#include <windows.h>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <vector>
/*
$env:Path = "C:\msys64\ucrt64\bin;" + $env:Path
& "C:\msys64\ucrt64\bin\g++.exe" projectver1.cpp -o projectver1.exe -std=c++17 -I"C:\Users\afra\Downloads\WinBGIm-64-1.0.1\WinBGIm-64-1.0.1\libbgi\include\bgi" -L"C:\Users\afra\Downloads\WinBGIm-64-1.0.1\WinBGIm-64-1.0.1\build\libbgi" -lbgi -lgdi32 -lcomdlg32 -luuid -loleaut32 -lole32
$LASTEXITCODE
.\projectver1.exe
*/

// ===== STAGE 1/3: window layout + world->screen transformation (proposal Eq.1-3) =====
const int SCREEN_W = 1200, SCREEN_H = 600; // whole window
const int VIEW_W = 912, VIEW_H = 532;      // city viewport (rest = side bar + bottom bar)
const double X_MIN = 0.0, X_MAX = 120.0, Y_MIN = 0.0, Y_MAX = 70.0;
const double SX = (VIEW_W - 1) / (X_MAX - X_MIN);
const double SY = (VIEW_H - 1) / (Y_MAX - Y_MIN);
const double GROUND_Y = 5.0;

int toScreenX(double wx) { return (int)floor(SX * (wx - X_MIN) + 0.5); }
int toScreenY(double wy) { return (VIEW_H - 1) - (int)floor(SY * (wy - Y_MIN) + 0.5); }
double screenToWorldX(int u) { return X_MIN + u / SX; }
double screenToWorldY(int v) { return Y_MIN + ((VIEW_H - 1) - v) / SY; }

#define PANEL_BG COLOR(15, 15, 25)

// ---------- physics / search constants ----------
const double G = 9.81;
const double PI = 3.14159265358979323846;
const double DEG2RAD = PI / 180.0, RAD2DEG = 180.0 / PI;
const double DT = 1.0 / 120.0; // animation step
const double SEARCH_DT = DT;   // MUST equal DT (the replay plays one recorded point per DT)
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
}; // (x,y) = bottom-left
struct Anchor
{
  double x, y;
};
struct Point2
{
  double x, y;
  Point2(double px = 0.0, double py = 0.0) : x(px), y(py) {}
};
struct TracePt // one recorded simulation step: position + attached anchor (-1 = flying)
{
  double x, y;
  int anchor;
  TracePt(double px = 0.0, double py = 0.0, int a = -1) : x(px), y(py), anchor(a) {}
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

double clampd(double v, double lo, double hi) { return v < lo ? lo : (v > hi ? hi : v); }

const int NUM_BUILDINGS = 5, NUM_ANCHORS = 4;
Building buildings[NUM_BUILDINGS];
Anchor anchors[NUM_ANCHORS] = {{26, 60}, {52, 60}, {78, 60}, {100, 60}};
Point2 startPoint(8, 56);
Point2 goalPoint(108, 25);
int goalBuilding = 4;
int currentLevel = 1;

bool roofReachable(int bi) { return buildings[bi].y + buildings[bi].h <= Y_MAX - 5.0; }

// put the goal on the roof of building bi, as close to wx as the roof allows
void setGoalOnBuilding(int bi, double wx)
{
  const Building &b = buildings[bi];
  double half = GOAL_EPS < b.w / 2 ? GOAL_EPS : b.w / 2;
  goalPoint.x = clampd(wx, b.x + half, b.x + b.w - half);
  goalPoint.y = b.y + b.h;
  goalBuilding = bi;
}

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
  setGoalOnBuilding(4, 108);
}

// ===== STAGE 6: collision detection =====
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

// ===== STAGE 5: projectile =====
void flightStep(Body &b, double dt)
{
  b.x += b.vx * dt;
  b.y += b.vy * dt - 0.5 * G * dt * dt;
  b.vy -= G * dt;
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
std::vector<TracePt> trace, bestTrace;
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
    trace.push_back(TracePt(b.x, b.y, ai));
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
    trace.push_back(TracePt(b.x, b.y, -1));
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
  trace.push_back(TracePt(startPoint.x, startPoint.y, -1));
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

// ===== STAGE 2/4/5/8: simulation state =====
enum Mode
{
  M_STAND,
  M_SWING,
  M_FLIGHT,
  M_CRASHED,
  M_REPLAY
};
Mode mode = M_STAND;
Body body;
Swing swing;
int selAnchor = 0, attachedAnchor = -1;
double releaseAngleDeg = 30.0, elapsed = 0.0, predLandX = 0.0, swingE0 = 0.0;
bool goalReached = false, hasPred = false;
char status[64], collisionMsg[32], searchMsg[96], hint[128];
std::vector<Point2> trail, predPath;
int trailCounter = 0;
bool keyDown[256], keyPressed[256];

bool paused = false, slowmo = false, showPaths = true, showDiag = false;
bool goalMode = false;
int hoverB = -1;
double hoverX = 0.0;
double animT = 0.0; // real-time clock for decorations
double fxT = 0.0;   // effect timer
int fxKind = 0;     // 0 attach, 1 release, 2 crash
Point2 fxPos;
size_t replayIdx = 0;

void startFx(int kind, double x, double y)
{
  fxT = 0.35;
  fxKind = kind;
  fxPos = Point2(x, y);
}

// reset the character, timer and HUD (the planned route is kept so X can replay it again)
void resetState()
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
  predPath.clear();
  trailCounter = 0;
  replayIdx = 0;
  fxT = 0;
  strcpy(status, "READY - SPACE to attach");
  strcpy(collisionMsg, "NONE");
}

void clearSearch()
{
  bestTrace.clear();
  hasBest = false;
  bestTime = 1e18;
  bestCount = 0;
  nodes = 0;
  routeText[0] = 0;
  strcpy(searchMsg, "Search: press ENTER");
}

void confirmGoal(int bi, double wx)
{
  if (!roofReachable(bi))
  {
    strcpy(status, "Roof out of reach");
    return;
  }
  setGoalOnBuilding(bi, wx);
  goalMode = false;
  resetState();
  clearSearch();
  sprintf(status, "Goal: building %d - ENTER", bi + 1);
}

void startReplay()
{
  if (!hasBest || bestTrace.size() < 2)
  {
    strcpy(status, "No route: press ENTER first");
    return;
  }
  resetState();
  mode = M_REPLAY;
  replayIdx = 0;
  strcpy(status, "REPLAY of planned route");
}

void tryAttach()
{
  if (!makeSwing(selAnchor, body, swing))
  {
    strcpy(status, "Anchor out of range");
    return;
  }
  if (!ropeClear(selAnchor, body))
  {
    strcpy(status, "Rope blocked by building");
    return;
  }
  attachedAnchor = selAnchor;
  mode = M_SWING;
  swingE0 = swingEnergy(swing);
  startFx(0, anchors[selAnchor].x, anchors[selAnchor].y);
  strcpy(status, "Swinging");
}

void releaseWeb()
{
  mode = M_FLIGHT;
  attachedAnchor = -1;
  predLandX = predictLandingX(body, GROUND_Y + BODY_R);
  hasPred = true;
  // predicted free-flight path from Eq.10-11 (ignores buildings)
  predPath.clear();
  for (double t = 0; t <= 8.0; t += 0.05)
  {
    double y = body.y + body.vy * t - 0.5 * G * t * t;
    predPath.push_back(Point2(body.x + body.vx * t, y));
    if (y <= GROUND_Y + BODY_R)
      break;
  }
  startFx(1, body.x, body.y);
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
      startFx(2, body.x, body.y);
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
      startFx(2, body.x, body.y);
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
  else if (mode == M_REPLAY)
  {
    replayIdx++;
    if (replayIdx >= bestTrace.size())
    {
      mode = M_STAND;
      attachedAnchor = -1;
      body.vx = body.vy = 0;
      goalReached = true;
      strcpy(status, "REPLAY done - GOAL REACHED");
      return;
    }
    const TracePt &p = bestTrace[replayIdx];
    const TracePt &q = bestTrace[replayIdx - 1];
    body.vx = (p.x - q.x) / DT;
    body.vy = (p.y - q.y) / DT;
    body.x = p.x;
    body.y = p.y;
    elapsed += DT;
    if (p.anchor != attachedAnchor)
      startFx(p.anchor >= 0 ? 0 : 1, p.anchor >= 0 ? anchors[p.anchor].x : p.x, p.anchor >= 0 ? anchors[p.anchor].y : p.y);
    attachedAnchor = p.anchor;
  }
  if ((mode == M_SWING || mode == M_FLIGHT || mode == M_REPLAY) && ++trailCounter % 4 == 0)
    trail.push_back(Point2(body.x, body.y));
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

// which building's roof is under world point (wx, wy)?  -1 = none
int buildingAtX(double wx, double wy)
{
  for (int i = 0; i < NUM_BUILDINGS; i++)
    if (wx >= buildings[i].x && wx <= buildings[i].x + buildings[i].w && wy <= buildings[i].y + buildings[i].h + 15.0)
      return i;
  return -1;
}

// ===== STAGE 1/2: decoration helpers =====
unsigned hash2(int a, int b)
{
  unsigned h = (unsigned)a * 374761393u + (unsigned)b * 668265263u;
  h = (h ^ (h >> 13)) * 1274126177u;
  return h ^ (h >> 16);
}
double wrapX(double x0, double v, double t) // position (m) of an object moving at v m/s, wrapping around the world
{
  double span = X_MAX + 24.0;
  double p = fmod(x0 + v * t, span);
  if (p < 0)
    p += span;
  return p - 12.0;
}
int lighten(int v, int a) { return v + a > 255 ? 255 : v + a; }

// centred text in a small bordered box
void labelBox(int cx, int top, const char *s, int fg, int bg)
{
  int w = 8 * (int)strlen(s);
  int h = 16;

  setfillstyle(SOLID_FILL, bg);
  bar(cx - w / 2 - 3, top, cx + w / 2 + 3, top + h + 4);

  setcolor(fg);
  rectangle(cx - w / 2 - 3, top, cx + w / 2 + 3, top + h + 4);

  setbkcolor(bg);
  outtextxy(cx - w / 2, top + 2, (char *)s);
  setbkcolor(BLACK);
}

// ===== STAGE 1/2: scene drawing (viewport = city only) =====
void drawSky()
{
  int groundTop = toScreenY(GROUND_Y);
  for (int sy = 0; sy < groundTop; sy += 4)
  {
    double t = (double)sy / groundTop;
    setfillstyle(SOLID_FILL, COLOR((int)(40 + 130 * t), (int)(80 + 130 * t), (int)(160 + 95 * t)));
    bar(0, sy, VIEW_W, sy + 4);
  }
}

void drawSkyline() // distant low-contrast silhouettes (visual only)
{
  int gt = toScreenY(GROUND_Y);
  setfillstyle(SOLID_FILL, COLOR(92, 122, 178));
  for (int k = 0; k < 16; k++)
  {
    unsigned h = hash2(k, 7);
    double wx = k * 8.0 + (h % 5);
    double w = 5.0 + (h >> 4) % 5;
    double ht = 12.0 + (h >> 8) % 26;
    bar(toScreenX(wx), toScreenY(GROUND_Y + ht), toScreenX(wx + w), gt);
  }
}

void drawClouds()
{
  const double cx0[3] = {10, 55, 95}, cy[3] = {64, 44, 66}, sp[3] = {1.2, 0.8, 1.6};
  setfillstyle(SOLID_FILL, COLOR(232, 238, 248));
  setcolor(COLOR(232, 238, 248));
  for (int i = 0; i < 3; i++)
  {
    int x = toScreenX(wrapX(cx0[i], sp[i], animT)), y = toScreenY(cy[i]);
    fillellipse(x, y, 30, 9);
    fillellipse(x - 20, y + 3, 18, 7);
    fillellipse(x + 22, y + 3, 20, 7);
    fillellipse(x + 2, y - 6, 16, 8);
  }
}

void drawBirds()
{
  const int ox[5] = {0, 14, 26, 8, 22}, oy[5] = {0, 6, -3, -10, 12};
  int bx = toScreenX(wrapX(-5, 5.0, animT));
  int by = toScreenY(48 + 3 * sin(animT * 0.6));
  setcolor(COLOR(30, 35, 55));
  for (int k = 0; k < 5; k++)
  {
    int x = bx + ox[k], y = by + oy[k];
    int w = (int)(4 * sin(animT * 9 + k * 1.7)); // wing flap
    line(x - 6, y - w, x, y);
    line(x, y, x + 6, y - w);
  }
}

void drawGround() // sidewalk + road (the physics ground is the top line, y = GROUND_Y)
{
  int gt = toScreenY(GROUND_Y);
  setfillstyle(SOLID_FILL, COLOR(150, 150, 156));
  bar(0, gt, VIEW_W, gt + 8);
  setfillstyle(SOLID_FILL, COLOR(45, 45, 52));
  bar(0, gt + 9, VIEW_W, VIEW_H);
  setcolor(COLOR(95, 95, 100));
  line(0, gt + 8, VIEW_W, gt + 8);
  setcolor(WHITE);
  line(0, gt, VIEW_W, gt);
  int ly = (gt + 9 + VIEW_H) / 2;
  setfillstyle(SOLID_FILL, COLOR(230, 210, 80));
  for (int x = 0; x < VIEW_W; x += 30)
    bar(x, ly, x + 16, ly + 1);
}

void drawBuildings()
{
  const int tone[NUM_BUILDINGS][3] = {{88, 100, 128}, {100, 112, 138}, {78, 92, 120}, {95, 106, 134}, {86, 98, 126}};
  for (int i = 0; i < NUM_BUILDINGS; i++)
  {
    int l = toScreenX(buildings[i].x), r = toScreenX(buildings[i].x + buildings[i].w);
    int t = toScreenY(buildings[i].y + buildings[i].h), bt = toScreenY(buildings[i].y);
    int cr = tone[i][0], cg = tone[i][1], cb = tone[i][2];
    if (i == goalBuilding)
    {
      cr = 60;
      cg = 118;
      cb = 112;
    } // goal building accent

    setfillstyle(SOLID_FILL, COLOR(cr, cg, cb)); // front face
    bar(l, t, r, bt);
    setfillstyle(SOLID_FILL, COLOR(cr * 7 / 10, cg * 7 / 10, cb * 7 / 10)); // darker side face
    bar(r - 6, t, r, bt);
    setfillstyle(SOLID_FILL, COLOR(lighten(cr, 45), lighten(cg, 45), lighten(cb, 45))); // roof edge
    bar(l, t, r, t + 3);

    int cols = (r - l - 14) / 13, rows = (bt - t - 40) / 17; // window grid
    int x0 = l + (r - l - 6 - cols * 13) / 2 + 3;
    for (int rr = 0; rr < rows; rr++)
      for (int c = 0; c < cols; c++)
      {
        bool lit = (hash2(i * 29 + c, rr * 7 + i) % 100) < 38;
        if (lit)
          setfillstyle(SOLID_FILL, COLOR(255, 214, 120));
        else
          setfillstyle(SOLID_FILL, COLOR(cr / 2, cg / 2, cb / 2 + 20));
        int wx = x0 + c * 13, wy = t + 12 + rr * 17;
        bar(wx, wy, wx + 7, wy + 9);
      }

    int mid = (l + r - 6) / 2; // ground floor: shop windows + door
    setfillstyle(SOLID_FILL, COLOR(150, 190, 220));
    bar(l + 6, bt - 14, mid - 10, bt - 5);
    bar(mid + 10, bt - 14, r - 12, bt - 5);
    setfillstyle(SOLID_FILL, COLOR(40, 42, 55));
    bar(mid - 6, bt - 17, mid + 6, bt);

    setcolor(BLACK);
    rectangle(l, t, r, bt);

    if (i % 2 == 0) // rooftop details (visual only)
    {
      setcolor(COLOR(40, 40, 50));
      line(l + 8, t, l + 8, t - 16);
      if (sin(animT * 4.0) > 0)
      {
        setcolor(RED);
        setfillstyle(SOLID_FILL, RED);
        fillellipse(l + 8, t - 18, 2, 2);
      }
    }
    else
    {
      setfillstyle(SOLID_FILL, COLOR(120, 86, 62));
      bar(l + 14, t - 10, l + 30, t);
      setcolor(BLACK);
      rectangle(l + 14, t - 10, l + 30, t);
    }
  }
}

void drawStreetLights()
{
  const double xs[5] = {6, 29, 48, 70, 91};
  int gt = toScreenY(GROUND_Y);
  for (int k = 0; k < 5; k++)
  {
    int x = toScreenX(xs[k]);
    setcolor(COLOR(30, 30, 36));
    line(x, gt, x, gt - 26);
    line(x + 1, gt, x + 1, gt - 26);
    line(x, gt - 26, x + 7, gt - 26);
    setcolor(COLOR(255, 230, 150));
    setfillstyle(SOLID_FILL, COLOR(255, 230, 150));
    fillellipse(x + 8, gt - 24, 3, 2);
  }
}

void drawCars() // decorative only
{
  const double x0[3] = {10, 50, 85}, v[3] = {9, -7, 6};
  const int lane[3] = {1, 0, 1};
  const int col[3][3] = {{200, 50, 50}, {240, 190, 50}, {60, 130, 210}};
  int gt = toScreenY(GROUND_Y);
  for (int k = 0; k < 3; k++)
  {
    int cx = toScreenX(wrapX(x0[k], v[k], animT));
    int by = lane[k] ? VIEW_H - 4 : gt + 22;
    setfillstyle(SOLID_FILL, COLOR(col[k][0], col[k][1], col[k][2]));
    bar(cx - 14, by - 8, cx + 14, by - 2);
    bar(cx - 8, by - 13, cx + 7, by - 8);
    setfillstyle(SOLID_FILL, COLOR(170, 210, 235));
    bar(cx - 6, by - 12, cx + 5, by - 9);
    setcolor(COLOR(20, 20, 20));
    setfillstyle(SOLID_FILL, COLOR(20, 20, 20));
    fillellipse(cx - 8, by - 2, 3, 3);
    fillellipse(cx + 8, by - 2, 3, 3);
  }
}

void drawGoalPad()
{
  const Building &b = buildings[goalBuilding];
  double half = GOAL_EPS < b.w / 2 ? GOAL_EPS : b.w / 2;
  int l = toScreenX(goalPoint.x - half), r = toScreenX(goalPoint.x + half), t = toScreenY(goalPoint.y);
  setfillstyle(SOLID_FILL, COLOR(40, 200, 90));
  bar(l, t - 3, r, t);
  setfillstyle(SOLID_FILL, COLOR(150, 255, 170));
  bar(l + 2, t - 1, r - 2, t);
  int px = (l + r) / 2;
  setcolor(WHITE);
  line(px, t - 3, px, t - 26);
  setcolor(RED);
  setfillstyle(SOLID_FILL, RED);
  int tri[6] = {px, t - 26, px + 14, t - 21, px, t - 16};
  fillpoly(3, tri);
  labelBox(px, t - 48, "GOAL", WHITE, COLOR(20, 90, 50));
}

void drawStartGoal()
{
  int sx = toScreenX(startPoint.x), sy = toScreenY(startPoint.y);
  setfillstyle(SOLID_FILL, COLOR(70, 80, 100)); // start platform
  bar(sx - 20, sy + 7, sx + 20, sy + 11);
  setcolor(GREEN);
  rectangle(sx - 20, sy + 7, sx + 20, sy + 11);
  labelBox(sx, sy + 16, "START", GREEN, COLOR(20, 25, 45));
  drawGoalPad();
}

void drawAnchors()
{
  char label[8];
  for (int i = 0; i < NUM_ANCHORS; i++)
  {
    int sx = toScreenX(anchors[i].x), sy = toScreenY(anchors[i].y);
    setcolor(BLACK);
    setfillstyle(SOLID_FILL, YELLOW);
    fillellipse(sx, sy, 6, 6);
    if (i == selAnchor)
    {
      setcolor(i == attachedAnchor ? GREEN : WHITE);
      circle(sx, sy, 11);
      circle(sx, sy, 12);
    }
    sprintf(label, "A%d", i + 1);
    labelBox(sx, sy - 34, label, YELLOW, COLOR(20, 25, 45));
  }
}

void drawGhost() // planned route: swing = cyan, flight = magenta (dashed)
{
  if (!showPaths || bestTrace.size() < 2)
    return;
  setlinestyle(SOLID_LINE, 0, 3);
  for (size_t i = 1; i < bestTrace.size(); i++)
  {
    if (((i >> 2) & 1) != 0)
      continue; // dash pattern
    setcolor(bestTrace[i].anchor >= 0 ? CYAN : LIGHTMAGENTA);
    line(toScreenX(bestTrace[i - 1].x), toScreenY(bestTrace[i - 1].y), toScreenX(bestTrace[i].x), toScreenY(bestTrace[i].y));
  }
  setlinestyle(SOLID_LINE, 0, 1);
}

void drawTrail() // actual path (fading) + predicted landing
{
  size_t n = trail.size();
  for (size_t i = 1; i < n; i++)
  {
    double f = 0.25 + 0.75 * (double)i / (double)n;
    setcolor(COLOR((int)(110 + 145 * f), (int)(150 - 10 * f), (int)(215 - 215 * f)));
    line(toScreenX(trail[i - 1].x), toScreenY(trail[i - 1].y), toScreenX(trail[i].x), toScreenY(trail[i].y));
  }
  if (showPaths && hasPred)
  {
    setcolor(LIGHTRED);
    for (size_t i = 1; i < predPath.size(); i++)
      if (i % 2 == 0)
        line(toScreenX(predPath[i - 1].x), toScreenY(predPath[i - 1].y), toScreenX(predPath[i].x), toScreenY(predPath[i].y));
    int px = toScreenX(predLandX), py = toScreenY(GROUND_Y);
    line(px - 5, py - 5, px + 5, py + 5);
    line(px - 5, py + 5, px + 5, py - 5);
  }
}

void drawRope()
{
  if ((mode != M_SWING && mode != M_REPLAY) || attachedAnchor < 0)
    return;
  setcolor(WHITE);
  line(toScreenX(anchors[attachedAnchor].x), toScreenY(anchors[attachedAnchor].y), toScreenX(body.x), toScreenY(body.y));
}

// local body frame -> screen: a = along "up", b = to the right
int localX(int cx, double ux, double uy, double a, double b) { return cx + (int)floor(a * ux + b * (-uy) + 0.5); }
int localY(int cy, double ux, double uy, double a, double b) { return cy + (int)floor(a * uy + b * ux + 0.5); }
void limb(int cx, int cy, double ux, double uy, double a1, double b1, double a2, double b2)
{
  int x1 = localX(cx, ux, uy, a1, b1), y1 = localY(cy, ux, uy, a1, b1);
  int x2 = localX(cx, ux, uy, a2, b2), y2 = localY(cy, ux, uy, a2, b2);
  line(x1, y1, x2, y2);
  line(x1 + 1, y1, x2 + 1, y2);
}

void drawCharacter()
{
  int cx = toScreenX(body.x), cy = toScreenY(body.y);
  double ux = 0, uy = -1; // "up" of the body in screen space
  bool roped = (mode == M_SWING || mode == M_REPLAY) && attachedAnchor >= 0;
  bool flying = (mode == M_FLIGHT || mode == M_CRASHED || (mode == M_REPLAY && !roped));
  if (roped)
  {
    double dx = toScreenX(anchors[attachedAnchor].x) - cx, dy = toScreenY(anchors[attachedAnchor].y) - cy;
    double d = sqrt(dx * dx + dy * dy);
    if (d > 1e-6)
    {
      ux = dx / d;
      uy = dy / d;
    }
  }
  else if (flying)
  {
    ux = clampd(body.vx * 0.04, -0.6, 0.6);
    uy = -sqrt(1.0 - ux * ux);
  }

  setcolor(COLOR(20, 50, 160)); // blue suit limbs
  if (roped)
  {
    limb(cx, cy, ux, uy, 4, -5, 17, -1.5);
    limb(cx, cy, ux, uy, 4, 5, 17, 1.5);
    limb(cx, cy, ux, uy, -5, -2, -16, -5);
    limb(cx, cy, ux, uy, -5, 2, -16, 5);
  }
  else if (flying)
  {
    limb(cx, cy, ux, uy, 3, -6, 10, -15);
    limb(cx, cy, ux, uy, 3, 6, 10, 15);
    limb(cx, cy, ux, uy, -5, -2, -14, -9);
    limb(cx, cy, ux, uy, -5, 2, -14, 9);
  }
  else
  {
    limb(cx, cy, ux, uy, -1, -7, -8, -11);
    limb(cx, cy, ux, uy, -1, 7, -8, 11);
    limb(cx, cy, ux, uy, -5, -3, -6, -10);
    limb(cx, cy, ux, uy, -5, 3, -6, 10);
  }
  setcolor(BLACK);
  setfillstyle(SOLID_FILL, RED);
  fillellipse(cx, cy, 7, 7);
  limb(cx, cy, ux, uy, -7, 0, 7, 0); // web pattern
  limb(cx, cy, ux, uy, 0, -7, 0, 7);
  setcolor(WHITE);
  setfillstyle(SOLID_FILL, WHITE);
  fillellipse(localX(cx, ux, uy, 2, -3), localY(cy, ux, uy, 2, -3), 2, 3);
  fillellipse(localX(cx, ux, uy, 2, 3), localY(cy, ux, uy, 2, 3), 2, 3);
}

void drawFx()
{
  if (fxT <= 0)
    return;
  int x = toScreenX(fxPos.x), y = toScreenY(fxPos.y);
  int rad = 6 + (int)((0.35 - fxT) * 90);
  setcolor(fxKind == 0 ? YELLOW : (fxKind == 1 ? CYAN : LIGHTRED));
  circle(x, y, rad);
  circle(x, y, rad / 2);
  if (fxKind == 2)
  {
    line(x - rad, y - rad, x + rad, y + rad);
    line(x - rad, y + rad, x + rad, y - rad);
  }
}

void drawGoalHover()
{
  if (!goalMode)
    return;
  char s[40];
  for (int i = 0; i < NUM_BUILDINGS; i++) // building numbers
  {
    sprintf(s, "%d", i + 1);
    labelBox(toScreenX(buildings[i].x + buildings[i].w / 2), toScreenY(buildings[i].y + buildings[i].h) - 32, s, WHITE, COLOR(20, 25, 45));
  }
  if (hoverB < 0)
    return;
  const Building &b = buildings[hoverB];
  double half = GOAL_EPS < b.w / 2 ? GOAL_EPS : b.w / 2;
  double gx = clampd(hoverX, b.x + half, b.x + b.w - half);
  int l = toScreenX(gx - half), r = toScreenX(gx + half), t = toScreenY(b.y + b.h);
  bool ok = roofReachable(hoverB);
  int col = ok ? YELLOW : LIGHTRED;
  setfillstyle(SOLID_FILL, col);
  bar(l, t - 4, r, t);
  if (ok)
    sprintf(s, "Click: building %d roof", hoverB + 1);
  else
    sprintf(s, "Building %d: out of reach", hoverB + 1);
  labelBox((l + r) / 2, t - 58, s, BLACK, col);
}

void drawBanner()
{
  if (!goalReached)
    return;
  char b[64];
  int cx = VIEW_W / 2, top = 40;
  int bg = COLOR(10, 70, 35);
  setfillstyle(SOLID_FILL, bg);
  bar(cx - 190, top, cx + 190, top + 74);
  setcolor(GREEN);
  rectangle(cx - 190, top, cx + 190, top + 74);
  setbkcolor(bg);
  setcolor(WHITE);
  settextstyle(DEFAULT_FONT, HORIZ_DIR, 2);
  const char *msg = "GOAL REACHED!";
  int w = 8 * (int)strlen(msg);
  outtextxy(cx - w / 2, top + 8, (char *)msg);
  settextstyle(DEFAULT_FONT, HORIZ_DIR, 1);
  if (hasBest)
    sprintf(b, "Actual %.2f s | Planned %.2f s", elapsed, bestTime);
  else
    sprintf(b, "Actual time %.2f s", elapsed);
  outtextxy(cx - 8 * (int)strlen(b) / 2, top + 48, b);
  setbkcolor(BLACK);
}

void drawScene()
{
  drawSky();
  drawSkyline();
  drawClouds();
  drawBirds();
  drawGround();
  drawBuildings();
  drawStreetLights();
  drawCars();
  drawStartGoal();
  drawGhost();
  drawTrail();
  drawAnchors();
  drawRope();
  drawCharacter();
  drawFx();
  drawGoalHover();
  drawBanner();
}

// ===== STAGE 8: HUD (side bar + bottom bar, full-window coordinates) =====
void hud(int x, int &y, const char *s, int fg = WHITE)
{
  char buf[36];
  strncpy(buf, s, 34);
  buf[34] = 0;
  setbkcolor(PANEL_BG);
  setcolor(fg);
  outtextxy(x, y, buf);
  setbkcolor(BLACK);
  y += 17;
}

void legend(int x, int &y, int color, const char *text, int style)
{
  setcolor(color);
  setlinestyle(style, 0, 1);
  line(x, y + 7, x + 26, y + 7);
  line(x, y + 8, x + 26, y + 8);
  setlinestyle(SOLID_LINE, 0, 1);
  int yy = y;
  hud(x + 34, yy, text);
  y += 17;
}

void drawSidebar()
{
  setfillstyle(SOLID_FILL, PANEL_BG);
  bar(VIEW_W, 0, SCREEN_W - 1, VIEW_H - 1);
  setcolor(COLOR(90, 100, 130));
  line(VIEW_W, 0, VIEW_W, VIEW_H - 1);

  const char *names[] = {"STAND", "SWING", "FLIGHT", "CRASHED", "REPLAY"};
  int x = VIEW_W + 8, y = 8;
  char b[96];

  hud(x, y, "SPIDER-SWING SIMULATOR", YELLOW);
  sprintf(b, "C233456 | Level %d", currentLevel);
  hud(x, y, b);
  y += 4;
  hud(x, y, "-- STATE --", LIGHTCYAN);
  sprintf(b, "Mode: %s%s%s", names[mode], paused ? " [PAUSED]" : "", slowmo ? " [SLOW]" : "");
  hud(x, y, b);
  double speed = sqrt(body.vx * body.vx + body.vy * body.vy);
  sprintf(b, "Time: %.2f s  Speed: %.1f m/s", elapsed, speed);
  hud(x, y, b);
  if (mode == M_SWING)
    sprintf(b, "Swing: %.1f deg | Rel: %.0f deg", swing.theta * RAD2DEG, releaseAngleDeg);
  else
    sprintf(b, "Swing: -- | Release: %.0f deg", releaseAngleDeg);
  hud(x, y, b);
  if (attachedAnchor >= 0)
  {
    double dx = body.x - anchors[attachedAnchor].x, dy = body.y - anchors[attachedAnchor].y;
    sprintf(b, "Rope length: %.1f m", sqrt(dx * dx + dy * dy));
  }
  else
    sprintf(b, "Rope length: --");
  hud(x, y, b);
  if (attachedAnchor >= 0)
    sprintf(b, "Anchor sel A%d | att A%d", selAnchor + 1, attachedAnchor + 1);
  else
    sprintf(b, "Anchor sel A%d | att --", selAnchor + 1);
  hud(x, y, b);
  sprintf(b, "Collision: %s", collisionMsg);
  hud(x, y, b);
  sprintf(b, "Status: %s", status);
  hud(x, y, b);

  y += 4;
  hud(x, y, "-- GOAL & SEARCH --", LIGHTCYAN);
  sprintf(b, "Goal: building %d roof", goalBuilding + 1);
  hud(x, y, b);
  sprintf(b, "Goal x = %.1f m, y = %.1f m", goalPoint.x, goalPoint.y);
  hud(x, y, b);
  hud(x, y, searchMsg);
  if (hasBest)
    sprintf(b, "Planned time: %.2f s", bestTime);
  else
    sprintf(b, "Planned time: --");
  hud(x, y, b);
  if (hasBest)
    sprintf(b, "Swings: %d | Flights: %ld", bestCount, nodes);
  else
    sprintf(b, "Swings: -- | Flights: %ld", nodes);
  hud(x, y, b);
  sprintf(b, "Route: %s", hasBest ? routeText : "--");
  hud(x, y, b);

  y += 4;
  if (showDiag)
  {
    hud(x, y, "-- DIAGNOSTICS [D] --", LIGHTCYAN);
    if (mode == M_SWING)
    {
      double e = swingEnergy(swing);
      double drift = fabs(swingE0) > 1e-9 ? 100.0 * fabs((e - swingE0) / swingE0) : 0.0;
      sprintf(b, "E/m: %.2f J/kg  drift %.3f %%", e, drift);
    }
    else
      sprintf(b, "E/m: --");
    hud(x, y, b);
    if (hasPred)
      sprintf(b, "Pred. landing x: %.1f m", predLandX);
    else
      sprintf(b, "Pred. landing x: --");
    hud(x, y, b);
    hud(x, y, "dt = 1/120 s   g = 9.81 m/s^2");
  }
  else
    hud(x, y, "D: show diagnostics", COLOR(150, 150, 170));

  y += 4;
  hud(x, y, "-- LEGEND --", LIGHTCYAN);
  legend(x, y, COLOR(255, 140, 0), "Actual trail", SOLID_LINE);
  legend(x, y, CYAN, "Plan: swing", DOTTED_LINE);
  legend(x, y, LIGHTMAGENTA, "Plan: flight", DOTTED_LINE);
  legend(x, y, LIGHTRED, "Predicted landing", DOTTED_LINE);
}

void updateHint()
{
  if (goalMode)
    strcpy(hint, "GOAL MODE: click a rooftop or press 1-5 (ESC cancels)");
  else if (paused)
    strcpy(hint, "PAUSED: P resumes, N advances 0.05 s");
  else if (mode == M_REPLAY)
    strcpy(hint, "Replaying the planned route (P pause, M slow motion, R stop)");
  else if (mode == M_CRASHED)
    strcpy(hint, "Crashed: press R to reset");
  else if (mode == M_SWING)
    strcpy(hint, "Swinging: SPACE releases now, or it auto-releases at the release angle");
  else if (mode == M_FLIGHT)
    strcpy(hint, "Flying: SPACE attaches to the selected anchor if it is in range");
  else if (goalReached)
    strcpy(hint, "Goal reached! R to try again, G to choose a new goal");
  else if (hasBest)
    strcpy(hint, "Route ready: X replays the full plan, SPACE tries the first swing yourself");
  else
    strcpy(hint, "SPACE attaches to the selected anchor, ENTER searches, G picks a goal");
}

void drawBottomBar()
{
  int bg = COLOR(20, 22, 34);
  setfillstyle(SOLID_FILL, bg);
  bar(0, VIEW_H, SCREEN_W - 1, SCREEN_H - 1);
  setcolor(COLOR(90, 100, 130));
  line(0, VIEW_H, SCREEN_W - 1, VIEW_H);
  setbkcolor(bg);
  setcolor(WHITE);
  outtextxy(8, VIEW_H + 5, (char *)"SPACE attach/release | LEFT/RIGHT anchor | UP/DOWN release angle | ENTER search | X replay | R reset");
  outtextxy(8, VIEW_H + 22, (char *)"G pick goal | P pause | N step | M slow-mo | V paths | D diagnostics | L level | T self-test | ESC exit");
  setcolor(YELLOW);
  outtextxy(8, VIEW_H + 42, hint);
  setbkcolor(BLACK);
}

void renderFrame()
{
  setviewport(0, 0, SCREEN_W - 1, SCREEN_H - 1, 1);
  setbkcolor(BLACK);
  cleardevice();
  setviewport(0, 0, VIEW_W - 1, VIEW_H - 1, 1); // clip the city drawing to its own area
  drawScene();
  setviewport(0, 0, SCREEN_W - 1, SCREEN_H - 1, 1);
  drawSidebar();
  drawBottomBar();
  swapbuffers();
}

// ===== STAGE 9: self-tests (printed in the console window) =====
void check(const char *name, bool ok) { printf("%-46s %s\n", name, ok ? "PASS" : "FAIL"); }
void selfTest()
{
  printf("\n===== SELF TEST =====\n");
  check("map (0,0)    -> bottom-left pixel", toScreenX(0) == 0 && toScreenY(0) == VIEW_H - 1);
  check("map (120,70) -> top-right pixel", toScreenX(120) == VIEW_W - 1 && toScreenY(70) == 0);
  check("screen->world->screen round trip", toScreenX(screenToWorldX(450)) == 450 && toScreenY(screenToWorldY(300)) == 300);

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

  int oldB = goalBuilding;
  Point2 oldG = goalPoint;
  setGoalOnBuilding(1, 200.0); // far to the right of the roof -> must be clamped onto it
  check("goal clamped onto the roof", goalPoint.x >= buildings[1].x && goalPoint.x <= buildings[1].x + buildings[1].w);
  goalBuilding = oldB;
  goalPoint = oldG;
  printf("===== END =====\n\n");
}

// ===== STAGE 2/9: main loop =====
int main()
{
  initwindow(SCREEN_W, SCREEN_H, (char *)"HERE COMES SPIDEYYYY!!!", 100, 100, true, false);
  loadLevel(currentLevel);
  resetState();
  clearSearch();

  clock_t last = clock();
  double acc = 0.0;
  while (true)
  {
    clock_t now = clock();
    double ft = (double)(now - last) / CLOCKS_PER_SEC;
    last = now;
    if (ft > 0.1)
      ft = 0.1;
    animT += ft;
    if (fxT > 0)
      fxT -= ft;

    updateKeys();
    if (keyPressed[VK_ESCAPE])
    {
      if (goalMode)
        goalMode = false;
      else
        break;
    }
    if (keyPressed['R'])
    {
      goalMode = false;
      resetState();
    }
    if (keyPressed['L'])
    {
      currentLevel = 3 - currentLevel;
      goalMode = false;
      loadLevel(currentLevel);
      resetState();
      clearSearch();
    }
    if (keyPressed['T'])
      selfTest();
    if (keyPressed['P'])
      paused = !paused;
    if (keyPressed['M'])
      slowmo = !slowmo;
    if (keyPressed['V'])
      showPaths = !showPaths;
    if (keyPressed['D'])
      showDiag = !showDiag;
    if (keyPressed['X'] && !goalMode)
      startReplay();
    if (keyPressed['G'])
    {
      goalMode = !goalMode;
      if (goalMode)
        strcpy(status, "Pick a goal: click or 1-5");
    }
    bool stepReq = paused && keyPressed['N'];

    if (keyPressed[VK_LEFT])
    {
      selAnchor = (selAnchor + NUM_ANCHORS - 1) % NUM_ANCHORS;
      if (mode == M_STAND)
        strcpy(status, "READY - SPACE to attach");
    }
    if (keyPressed[VK_RIGHT])
    {
      selAnchor = (selAnchor + 1) % NUM_ANCHORS;
      if (mode == M_STAND)
        strcpy(status, "READY - SPACE to attach");
    }
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
    if (keyPressed[VK_SPACE] && !goalMode)
    {
      if (mode == M_SWING)
        releaseWeb();
      else if (mode == M_STAND || mode == M_FLIGHT)
        tryAttach();
    }

    if (goalMode) // goal selection: mouse hover + click, or keys 1-5
    {
      hoverB = -1;
      int mx = mousex(), my = mousey();
      if (mx >= 0 && mx < VIEW_W && my >= 0 && my < VIEW_H)
      {
        hoverX = screenToWorldX(mx);
        hoverB = buildingAtX(hoverX, screenToWorldY(my));
      }
      if (ismouseclick(WM_LBUTTONDOWN))
      {
        int cx, cy;
        getmouseclick(WM_LBUTTONDOWN, cx, cy);
        if (cx >= 0 && cx < VIEW_W && cy >= 0 && cy < VIEW_H)
        {
          double wx = screenToWorldX(cx);
          int bi = buildingAtX(wx, screenToWorldY(cy));
          if (bi >= 0)
            confirmGoal(bi, wx);
          else
            strcpy(status, "Not a rooftop - try again");
        }
      }
      for (int k = 0; k < NUM_BUILDINGS; k++)
        if (goalMode && keyPressed['1' + k])
          confirmGoal(k, buildings[k].x + buildings[k].w / 2);
    }
    else
      clearmouseclick(WM_LBUTTONDOWN);

    if (keyPressed[VK_RETURN] && !goalMode)
    {
      resetState(); // the plan always starts from START
      strcpy(searchMsg, "Search: RUNNING...");
      updateHint();
      renderFrame(); // show the message before blocking
      clock_t c0 = clock();
      bool ok = findBestRoute();
      double secs = (double)(clock() - c0) / CLOCKS_PER_SEC;
      if (ok)
      {
        sprintf(searchMsg, "Search: found (%.2f s compute)", secs);
        selAnchor = bestSteps[0].anchor;
        releaseAngleDeg = bestSteps[0].angleDeg;
        strcpy(status, "Route found - X to replay");
      }
      else
      {
        strcpy(searchMsg, "No route within search limits");
        strcpy(status, "No route found");
      }
      last = clock();
      acc = 0;
    }

    double scale = slowmo ? 0.25 : 1.0;
    if (!paused)
      acc += ft * scale;
    if (stepReq)
      for (int k = 0; k < 6; k++)
        stepPhysics(); // 6 x 1/120 s = 0.05 s
    while (acc >= DT)
    {
      stepPhysics();
      acc -= DT;
    } // fixed time step

    updateHint();
    renderFrame();
    delay(8);
  }
  closegraph();
  return 0;
}