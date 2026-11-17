// Physics-Based Web-Swinging Trajectory Optimization and Collision Avoidance Simulator
// g++ project.cpp -o project.exe -std=c++17 -lbgi -lgdi32 -lcomdlg32 -luuid -loleaut32 -lole32

#include <graphics.h>
#include <windows.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <vector>
/*
$env:Path = "C:\msys64\ucrt64\bin;" + $env:Path
& "C:\msys64\ucrt64\bin\g++.exe" project.cpp -o project.exe -std=c++17 -I"C:\Users\afra\Downloads\WinBGIm-64-1.0.1\WinBGIm-64-1.0.1\libbgi\include\bgi" -L"C:\Users\afra\Downloads\WinBGIm-64-1.0.1\WinBGIm-64-1.0.1\build\libbgi" -lbgi -lgdi32 -lcomdlg32 -luuid -loleaut32 -lole32
$LASTEXITCODE
.\project.exe
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
#define ROUTE_COL COLOR(0, 235, 255)
#define RED_SUIT COLOR(205, 25, 35)
#define BLUE_SUIT COLOR(30, 70, 200)

// ---------- physics / search constants ----------
const double G = 9.81;
const double PI = 3.14159265358979323846;
const double DEG2RAD = PI / 180.0, RAD2DEG = 180.0 / PI;
const double DT = 1.0 / 120.0; // animation step
const double SEARCH_DT = DT;   // MUST equal DT (the replay plays one recorded point per DT)
const double BODY_R = 0.8;
const double LMIN = 8.0, LMAX = 40.0;
const double GOAL_HALF_MAX = 3.0; // [FIXED] half-width (m) of the landing strip -> strip is 6 m wide
const int MAX_SWINGS = 3;
const double SWING_MAX_T = 6.0, FLIGHT_MAX_T = 6.0;
const long MAX_NODES = 60000; // [NEW] search effort limits
const double MAX_SEARCH_SEC = 4.0;

int swingHalfCycles = 0;

// [FIXED] release angles every 5 degrees (old list started at -30 and could not reach buildings 1-2)
const int NUM_ANGLES = 29;
const double RELEASE_ANGLES[NUM_ANGLES] = {
    -70, -65, -60, -55, -50, -45, -40, -35, -30, -25, -20, -15, -10, -5, 0,
    5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70};
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
double goalHalf = GOAL_HALF_MAX; // [FIXED] half-width of the CURRENT landing strip
int currentLevel = 1;

bool roofReachable(int bi) { return buildings[bi].y + buildings[bi].h <= Y_MAX - 5.0; }

// [FIXED] the goal is a landing strip centred at goalPoint.x on the roof of building bi
void setGoalOnBuilding(int bi, double wx)
{
  const Building &b = buildings[bi];
  goalHalf = GOAL_HALF_MAX < b.w / 2 ? GOAL_HALF_MAX : b.w / 2;
  goalPoint.x = clampd(wx, b.x + goalHalf, b.x + b.w - goalHalf);
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
int landedOn = -2; // [NEW] roof index of the last landing (-1 = ground, -2 = none)

// In flight: touching a roof/ground from above = landing, anything else = crash
Contact flightContact(Body &b)
{
  landedOn = -2;
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
        landedOn = i;
        return C_LAND;
      }
      return C_CRASH;
    }
  }
  if (b.y - BODY_R <= GROUND_Y)
  {
    b.y = GROUND_Y + BODY_R;
    b.vx = b.vy = 0;
    landedOn = -1;
    return C_LAND;
  }
  return C_NONE;
}

// [FIXED] success = landed on the SELECTED goal roof AND inside its landing strip
bool atGoal(const Body &b)
{
  return landedOn == goalBuilding && fabs(b.x - goalPoint.x) <= goalHalf;
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
bool searchAborted = false; // [NEW] effort limit reached
clock_t searchStart = 0;
RouteStep curSteps[MAX_SWINGS], bestSteps[MAX_SWINGS];
std::vector<TracePt> trace, bestTrace; // bestTrace = FULL route: origin, every swing, every flight, landing
char routeText[128] = "";

void searchFlight(Body b, double t0, int depth, int fromAnchor);

void searchSwing(int ai, const Body &start, double t0, int depth)
{
  if (searchAborted || t0 >= bestTime)
    return;
  Swing s;
  if (!makeSwing(ai, start, s) || !ropeClear(ai, start))
    return;
  size_t mark = trace.size();
  double t = t0, prevTheta = s.theta;
  Body b = start;
  while (t - t0 < SWING_MAX_T && t < bestTime && !searchAborted)
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
  if (searchAborted || t0 >= bestTime)
    return;
  nodes++;
  if (nodes > MAX_NODES ||
      ((nodes & 15) == 0 && (double)(clock() - searchStart) / CLOCKS_PER_SEC > MAX_SEARCH_SEC))
  {
    searchAborted = true;
    return;
  }
  // pruning: even at the maximum possible speed (energy bound) the goal strip cannot be reached sooner
  double hDrop = b.y - (GROUND_Y + BODY_R);
  if (hDrop < 0)
    hDrop = 0;
  double vmax = sqrt(b.vx * b.vx + b.vy * b.vy + 2.0 * G * hDrop);
  double dxg = fabs(goalPoint.x - b.x) - goalHalf; // [FIXED] uses the SELECTED goal strip
  if (dxg < 0)
    dxg = 0;
  if (vmax > 1e-9 && t0 + dxg / vmax >= bestTime)
    return;

  bool done[NUM_ANCHORS][NUM_ATTACH] = {};
  size_t mark = trace.size();
  double t = t0;
  while (t - t0 < FLIGHT_MAX_T && t < bestTime && !searchAborted)
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
    { // valid route into the goal strip, faster than best
      bestTime = t;
      bestCount = depth;
      hasBest = true;
      for (int i = 0; i < depth; i++)
        bestSteps[i] = curSteps[i];
      bestTrace = trace; // [FIXED] full trajectory copy; nothing clears it until the next search
    }
    break; // landed elsewhere or crashed
  }
  trace.resize(mark);
}

// [FIXED] searches from ANY origin: s0 at rest (inAir=false) or s0 flying/released (inAir=true)
bool findBestRoute(const Body &s0, bool inAir, int fromAnchor)
{
  bestTime = 1e18;
  hasBest = false;
  bestCount = 0;
  nodes = 0;
  searchAborted = false;
  searchStart = clock();
  trace.clear();
  bestTrace.clear();
  routeText[0] = 0;
  trace.push_back(TracePt(s0.x, s0.y, -1)); // origin is the first point of the route
  if (inAir)
    searchFlight(s0, 0.0, 0, fromAnchor);
  else
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
char status[64], collisionMsg[32], searchMsg[96], hint[160], runRoute[128];
int swingsUsed = 0;
std::vector<Point2> trail, predPath;
int trailCounter = 0;
bool keyDown[256], keyPressed[256];

bool paused = false, slowmo = false, showPaths = true, showDiag = false;
bool goalMode = false;
int hoverB = -1;
double hoverX = 0.0;
double animT = 0.0; // real-time clock for decorations
size_t replayIdx = 0;

// [NEW] effect state (visual only: never touches physics, collision or search)
double fxT = 0.0; // web attach / release burst timer
int fxKind = 0;   // 0 attach, 1 release
Point2 fxPos;
double crashT = 0.0; // comic crash burst timer
Point2 crashPos;
int crashPhrase = 0;
double celebT = -1.0; // goal celebration clock (-1 = off)
Point2 celebPos;
const int NUM_PHRASES = 8;
const char *const PHRASES[NUM_PHRASES] = {
    "OOPS!", "THWIP... CRASH!", "WEB-SLIP!", "NOT AGAIN!", "SPIDEY DOWN!", "BOOM!", "SMASH!", "POW!"};

void startFx(int kind, double x, double y)
{
  fxT = 0.35;
  fxKind = kind;
  fxPos = Point2(x, y);
}
void startCrash(double x, double y)
{
  crashT = 2.2;
  crashPos = Point2(x, y);
  crashPhrase = rand() % NUM_PHRASES;
}
void markGoalReached()
{
  goalReached = true;
  celebT = 0.0;
  celebPos = Point2(body.x, body.y);
  strcpy(status, "GOAL REACHED!");
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
  crashT = 0;
  celebT = -1.0;
  swingsUsed = 0;
  runRoute[0] = 0;
  landedOn = -2;
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
  searchAborted = false;
  routeText[0] = 0;
  strcpy(searchMsg, "Search: press ENTER");
}

// [FIXED] choosing a goal keeps the character where it is; it only clears the old plan / success state
void confirmGoal(int bi, double wx)
{
  if (!roofReachable(bi))
  {
    strcpy(status, "Roof out of reach");
    return;
  }
  setGoalOnBuilding(bi, wx);
  goalMode = false;
  goalReached = false;
  celebT = -1.0;
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
  body.x = bestTrace[0].x; // the plan may start away from START
  body.y = bestTrace[0].y;
  mode = M_REPLAY;
  replayIdx = 0;
  swingsUsed = bestCount;
  strcpy(runRoute, routeText);
  strcpy(status, "REPLAY of planned route");
}

void tryAttach()
{
  swingHalfCycles = 0;
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
  swingsUsed++;
  char tmp[16];
  sprintf(tmp, "%sA%d", runRoute[0] ? " > " : "", selAnchor + 1);
  if (strlen(runRoute) < 100)
    strcat(runRoute, tmp);
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
  swingHalfCycles = 0;
  if (mode == M_SWING)
  {
    double prev = swing.theta;
    swingStep(swing, DT);
    swingPos(swing, body);
    elapsed += DT;
    if (swingCrash(body, attachedAnchor))
    {
      mode = M_CRASHED;
      startCrash(body.x, body.y);
      strcpy(collisionMsg, "CRASH (swing/rope)");
      strcpy(status, "CRASHED - PRESS R");
      return;
    }
    double th = releaseAngleDeg * DEG2RAD;
    bool crossed = (prev < th && swing.theta >= th) ||
                   (prev > th && swing.theta <= th);

    // Count each pass through the lowest point (theta = 0).
    if (prev < 0.0 && swing.theta >= 0.0)
      swingHalfCycles++;

    // Allow automatic release only after completing one half-swing.
    if (crossed && body.vx > 0 && swingHalfCycles >= 1)
      releaseWeb();
  }
  else if (mode == M_FLIGHT)
  {
    flightStep(body, DT);
    elapsed += DT;
    Contact c = flightContact(body);
    if (c == C_CRASH)
    {
      mode = M_CRASHED;
      startCrash(body.x, body.y);
      strcpy(collisionMsg, "CRASH (building/edge)");
      strcpy(status, "CRASHED - PRESS R");
    }
    else if (c == C_LAND)
    {
      mode = M_STAND;
      if (atGoal(body))
        markGoalReached();
      else
        strcpy(status, "Landed (not in goal strip)");
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
      markGoalReached();
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

// ===== STAGE 1/2: decoration helpers (visual only) =====
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

// [FIXED] centred text box: no textwidth()/textheight(); 8 px per char, 16 px tall
void labelBox(int cx, int top, const char *s, int fg, int bg)
{
  int w = 8 * (int)strlen(s);
  int hw = w / 2 + 4;
  if (cx < hw)
    cx = hw;
  if (cx > VIEW_W - hw)
    cx = VIEW_W - hw;
  setfillstyle(SOLID_FILL, bg);
  bar(cx - w / 2 - 3, top, cx + w / 2 + 3, top + 20);
  setcolor(fg);
  rectangle(cx - w / 2 - 3, top, cx + w / 2 + 3, top + 20);
  setbkcolor(bg);
  outtextxy(cx - w / 2, top + 2, (char *)s);
  setbkcolor(BLACK);
}

// [NEW] spider-web net: straight spokes + straight ring segments
void drawWebNet(int cx, int cy, int R, int spokes, int rings, int col)
{
  setcolor(col);
  for (int s = 0; s < spokes; s++)
  {
    double a = 2.0 * PI * s / spokes;
    line(cx, cy, cx + (int)(R * cos(a)), cy - (int)(R * sin(a)));
  }
  for (int r = 1; r <= rings; r++)
  {
    double rr = (double)R * r / rings;
    for (int s = 0; s < spokes; s++)
    {
      double a1 = 2.0 * PI * s / spokes, a2 = 2.0 * PI * (s + 1) / spokes;
      line(cx + (int)(rr * cos(a1)), cy - (int)(rr * sin(a1)), cx + (int)(rr * cos(a2)), cy - (int)(rr * sin(a2)));
    }
  }
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

    if (i != goalBuilding) // rooftop details (visual only) - never on the goal roof
    {
      if (i % 2 == 0)
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
}

void drawStreetLights()
{
  const double xs[5] = {29, 48, 70, 91, 118};
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

void drawMarvelBanner() // [NEW] roadside advertising board in the free gap left of building 1 (decorative only)
{
  int gt = toScreenY(GROUND_Y);
  int x0 = 1, x1 = 103, yt = gt - 92, yb = gt - 52; // board; building 1 starts at x = 106 px
  int board = COLOR(200, 20, 30);
  setcolor(COLOR(60, 60, 70));
  setlinestyle(SOLID_LINE, 0, 3);
  line(18, yb, 18, gt);
  line(86, yb, 86, gt);
  setlinestyle(SOLID_LINE, 0, 1);
  setfillstyle(SOLID_FILL, board);
  bar(x0, yt, x1, yb);
  setcolor(WHITE);
  rectangle(x0, yt, x1, yb);
  setbkcolor(board);
  settextstyle(DEFAULT_FONT, HORIZ_DIR, 2);
  outtextxy(x0 + 3 + (x1 - x0 - 6 - 96) / 2, yt + 4, (char *)"MARVEL"); // 6 chars * 16 px = 96 px
  settextstyle(DEFAULT_FONT, HORIZ_DIR, 1);
  setbkcolor(BLACK);
  setfillstyle(SOLID_FILL, COLOR(60, 60, 70)); // small foot plates
  bar(12, gt - 3, 24, gt);
  bar(80, gt - 3, 92, gt);
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

void drawGoalPad() // [FIXED] draws the REAL landing strip (same width the success test uses)
{
  int l = toScreenX(goalPoint.x - goalHalf), r = toScreenX(goalPoint.x + goalHalf), t = toScreenY(goalPoint.y);
  setfillstyle(SOLID_FILL, COLOR(40, 200, 90));
  bar(l, t - 4, r, t);
  setfillstyle(SOLID_FILL, COLOR(150, 255, 170));
  bar(l + 2, t - 2, r - 2, t - 1);
  setcolor(WHITE);
  line(l, t - 4, l, t - 12);
  line(r, t - 4, r, t - 12);
  int px = (l + r) / 2;
  line(px, t - 4, px, t - 28);
  setcolor(RED);
  setfillstyle(SOLID_FILL, RED);
  int tri[6] = {px, t - 28, px + 14, t - 23, px, t - 18};
  fillpoly(3, tri);
  labelBox(px, t - 52, "GOAL", WHITE, COLOR(20, 90, 50));
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

// [FIXED] planned route: cyan dots from the origin through every swing and flight to the landing.
// Drawn above buildings/background and below the character. Toggle with V.
void drawGhost()
{
  if (!showPaths || bestTrace.size() < 2)
    return;
  size_t n = bestTrace.size();
  setcolor(COLOR(0, 150, 165)); // faint rope previews at every attach / release moment
  for (size_t i = 1; i < n; i++)
  {
    int a0 = bestTrace[i - 1].anchor, a1 = bestTrace[i].anchor;
    if (a0 == a1)
      continue;
    if (a1 >= 0)
      line(toScreenX(anchors[a1].x), toScreenY(anchors[a1].y), toScreenX(bestTrace[i].x), toScreenY(bestTrace[i].y));
    if (a0 >= 0)
      line(toScreenX(anchors[a0].x), toScreenY(anchors[a0].y), toScreenX(bestTrace[i - 1].x), toScreenY(bestTrace[i - 1].y));
  }
  setcolor(ROUTE_COL);
  setfillstyle(SOLID_FILL, ROUTE_COL);
  int lastX = -1000, lastY = -1000; // one dot every ~7 pixels along the whole path
  for (size_t i = 0; i < n; i++)
  {
    int x = toScreenX(bestTrace[i].x), y = toScreenY(bestTrace[i].y);
    int dx = x - lastX, dy = y - lastY;
    if (dx * dx + dy * dy >= 49 || i + 1 == n)
    {
      fillellipse(x, y, 2, 2);
      lastX = x;
      lastY = y;
    }
  }
  circle(toScreenX(bestTrace[n - 1].x), toScreenY(bestTrace[n - 1].y), 7); // planned landing
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

void drawRope() // straight white web rope
{
  if ((mode != M_SWING && mode != M_REPLAY) || attachedAnchor < 0)
    return;
  setcolor(WHITE);
  line(toScreenX(anchors[attachedAnchor].x), toScreenY(anchors[attachedAnchor].y), toScreenX(body.x), toScreenY(body.y));
  line(toScreenX(anchors[attachedAnchor].x) + 1, toScreenY(anchors[attachedAnchor].y), toScreenX(body.x) + 1, toScreenY(body.y));
}

// ---------- [NEW] Spider-Man style character (vector art in a rotated local frame) ----------
struct CharFrame
{
  int cx, cy;
  double ux, uy, off;
}; // (ux,uy) = body "up" in screen space
const double CHAR_K = 0.75;
// local (a = along up, b = to the right) -> screen
int cfx(const CharFrame &f, double a, double b) { return f.cx + (int)floor((a * CHAR_K + f.off) * f.ux + b * CHAR_K * (-f.uy) + 0.5); }
int cfy(const CharFrame &f, double a, double b) { return f.cy + (int)floor((a * CHAR_K + f.off) * f.uy + b * CHAR_K * f.ux + 0.5); }

void cLimb(const CharFrame &f, double a1, double b1, double a2, double b2, int col)
{
  setcolor(col);
  setlinestyle(SOLID_LINE, 0, 3);
  line(cfx(f, a1, b1), cfy(f, a1, b1), cfx(f, a2, b2), cfy(f, a2, b2));
  setlinestyle(SOLID_LINE, 0, 1);
}
void cLine(const CharFrame &f, double a1, double b1, double a2, double b2, int col)
{
  setcolor(col);
  line(cfx(f, a1, b1), cfy(f, a1, b1), cfx(f, a2, b2), cfy(f, a2, b2));
}
void cBall(const CharFrame &f, double a, double b, int r, int fill)
{
  setcolor(BLACK);
  setfillstyle(SOLID_FILL, fill);
  fillellipse(cfx(f, a, b), cfy(f, a, b), r, r);
}
void cQuad(const CharFrame &f, double a1, double b1, double a2, double b2, double a3, double b3, double a4, double b4, int fill)
{
  int p[8] = {cfx(f, a1, b1), cfy(f, a1, b1), cfx(f, a2, b2), cfy(f, a2, b2),
              cfx(f, a3, b3), cfy(f, a3, b3), cfx(f, a4, b4), cfy(f, a4, b4)};
  setfillstyle(SOLID_FILL, fill);
  setcolor(BLACK);
  fillpoly(4, p);
}

void drawCharacter()
{
  CharFrame f;
  f.cx = toScreenX(body.x);
  f.cy = toScreenY(body.y);
  f.ux = 0.0;
  f.uy = -1.0;
  f.off = 11.0; // standing: feet rest on the bottom of the collision circle
  int pose = 0; // 0 standing, 1 swinging, 2 flying
  double lean = 0.0;
  bool roped = (mode == M_SWING || mode == M_REPLAY) && attachedAnchor >= 0;
  bool flying = (mode == M_FLIGHT || mode == M_CRASHED || (mode == M_REPLAY && !roped));
  if (roped)
  {
    double dx = toScreenX(anchors[attachedAnchor].x) - f.cx, dy = toScreenY(anchors[attachedAnchor].y) - f.cy;
    double d = sqrt(dx * dx + dy * dy);
    if (d > 1e-6)
    {
      f.ux = dx / d;
      f.uy = dy / d;
    } // body tilts along the rope = follows the swing
    f.off = 2.0;
    pose = 1;
    lean = clampd(body.vx / 12.0, -1.0, 1.0);
  }
  else if (flying)
  {
    f.ux = clampd(body.vx * 0.04, -0.6, 0.6);
    f.uy = -sqrt(1.0 - f.ux * f.ux);
    f.off = 2.0;
    pose = 2;
    lean = clampd(body.vx / 12.0, -1.0, 1.0);
  }

  for (int s = -1; s <= 1; s += 2) // blue legs + red boots
  {
    double fa, fb;
    if (pose == 0)
    {
      fa = -19;
      fb = s * 5;
    }
    else if (pose == 1)
    {
      fa = -17;
      fb = s * 3 - lean * 7;
    }
    else
    {
      fa = -15;
      fb = s * 7 - lean * 3;
    }
    cLimb(f, -8, s * 2.5, fa, fb, BLUE_SUIT);
    cBall(f, fa, fb, 3, RED_SUIT);
  }
  cQuad(f, 3, -5, 3, 5, -6, 3.5, -6, -3.5, RED_SUIT);      // red torso
  cQuad(f, -6, -3.5, -6, 3.5, -10, 3, -10, -3, BLUE_SUIT); // blue lower body
  cLine(f, 3, 0, -6, 0, BLACK);                            // web lines on the suit
  cLine(f, -4, -4.2, -4, 4.2, BLACK);
  cBall(f, 0, 0, 2, BLACK); // spider emblem
  for (int s = -1; s <= 1; s += 2)
  {
    cLine(f, 0, 0, 2.5, s * 5, BLACK);
    cLine(f, 0, 0, 0, s * 5, BLACK);
    cLine(f, 0, 0, -2.5, s * 5, BLACK);
  }
  for (int s = -1; s <= 1; s += 2) // red arms + red gloves
  {
    double ha, hb;
    if (pose == 0)
    {
      ha = -7;
      hb = s * 9;
    }
    else if (pose == 1)
    {
      ha = 17;
      hb = s * 1.5;
    }
    else
    {
      ha = 11;
      hb = s * 11;
    }
    cLimb(f, 1, s * 5, ha, hb, RED_SUIT);
    cBall(f, ha, hb, 2, RED_SUIT);
  }
  cBall(f, 9, 0, 6, RED_SUIT);    // head
  cLine(f, 9, 0, 14.5, 0, BLACK); // mask web lines
  cLine(f, 9, 0, 13, 5, BLACK);
  cLine(f, 9, 0, 13, -5, BLACK);
  cLine(f, 9, 0, 4.5, 5, BLACK);
  cLine(f, 9, 0, 4.5, -5, BLACK);
  setcolor(BLACK); // large white eyes, black outline
  setfillstyle(SOLID_FILL, WHITE);
  for (int s = -1; s <= 1; s += 2)
    fillellipse(cfx(f, 10, s * 4), cfy(f, 10, s * 4), 3, 4);
}

// [NEW] brief web effects: attach / release burst
void drawFx()
{
  if (fxT <= 0)
    return;
  int x = toScreenX(fxPos.x), y = toScreenY(fxPos.y);
  int rad = 6 + (int)((0.35 - fxT) * 100);
  drawWebNet(x, y, rad, 8, 2, fxKind == 0 ? WHITE : ROUTE_COL);
}

// [NEW] goal celebration: growing web net + expanding ring + short flash (stays until R)
void drawCelebration()
{
  if (!goalReached || celebT < 0)
    return;
  int cx = toScreenX(celebPos.x), cy = toScreenY(celebPos.y);
  double grow = celebT < 0.6 ? celebT / 0.6 : 1.0;
  int R = (int)(72 * grow);
  if (R >= 6)
    drawWebNet(cx, cy, R, 12, 3, COLOR(245, 248, 255));
  if (celebT < 1.2)
  {
    int rr = 12 + (int)(celebT * 150);
    setcolor(YELLOW);
    circle(cx, cy, rr);
    circle(cx, cy, rr + 3);
    setcolor(LIGHTGREEN);
    circle(cx, cy, rr / 2);
  }
  if (celebT < 0.3)
  {
    setcolor(WHITE);
    rectangle(1, 1, VIEW_W - 2, VIEW_H - 2);
    rectangle(2, 2, VIEW_W - 3, VIEW_H - 3);
    rectangle(3, 3, VIEW_W - 4, VIEW_H - 4);
  }
}

// [NEW] comic-style crash burst with a random Spider-Man phrase (temporary)
void drawCrashBurst()
{
  if (crashT <= 0)
    return;
  const char *txt = PHRASES[crashPhrase];
  int len = (int)strlen(txt);
  int size = (16 * len <= 112) ? 2 : 1;
  int tw = (size == 2 ? 16 : 8) * len, th = (size == 2 ? 32 : 16);
  int R = tw / 2 + 34;
  double age = 2.2 - crashT;
  double sc = age < 0.2 ? 0.4 + 3.0 * age : 1.0; // quick pop-in
  int cx = toScreenX(crashPos.x), cy = toScreenY(crashPos.y) - R - 8;
  if (cx < R + 4)
    cx = R + 4;
  if (cx > VIEW_W - R - 4)
    cx = VIEW_W - R - 4;
  if (cy < R + 4)
    cy = R + 4;
  int Ro = (int)(R * sc), Ri = (int)(R * sc * 0.68);
  int pts[48], sh[48];
  for (int i = 0; i < 24; i++)
  {
    double a = PI * i / 12.0 + 0.13;
    int rr = (i % 2 == 0) ? Ro : Ri;
    pts[2 * i] = cx + (int)(rr * cos(a));
    pts[2 * i + 1] = cy - (int)(rr * sin(a));
    sh[2 * i] = pts[2 * i] + 4;
    sh[2 * i + 1] = pts[2 * i + 1] + 4;
  }
  int burst = COLOR(255, 225, 40);
  setcolor(BLACK);
  setfillstyle(SOLID_FILL, BLACK);
  fillpoly(24, sh); // hard comic shadow
  setcolor(COLOR(220, 30, 40));
  setfillstyle(SOLID_FILL, burst);
  fillpoly(24, pts);
  if (sc >= 1.0)
  {
    setbkcolor(burst);
    setcolor(COLOR(215, 25, 35));
    settextstyle(DEFAULT_FONT, HORIZ_DIR, size);
    outtextxy(cx - tw / 2, cy - th / 2, (char *)txt);
    settextstyle(DEFAULT_FONT, HORIZ_DIR, 1);
    setbkcolor(BLACK);
  }
}

void drawGoalHover()
{
  if (!goalMode)
    return;
  char s[48];
  for (int i = 0; i < NUM_BUILDINGS; i++) // building numbers
  {
    sprintf(s, "%d", i + 1);
    labelBox(toScreenX(buildings[i].x + buildings[i].w / 2), toScreenY(buildings[i].y + buildings[i].h) - 34, s, WHITE, COLOR(20, 25, 45));
  }
  if (hoverB < 0)
    return;
  const Building &b = buildings[hoverB];
  double half = GOAL_HALF_MAX < b.w / 2 ? GOAL_HALF_MAX : b.w / 2;
  double gx = clampd(hoverX, b.x + half, b.x + b.w - half);
  int l = toScreenX(gx - half), r = toScreenX(gx + half), t = toScreenY(b.y + b.h);
  bool ok = roofReachable(hoverB);
  int col = ok ? YELLOW : LIGHTRED;
  setfillstyle(SOLID_FILL, col);
  bar(l, t - 5, r, t);
  if (ok)
    sprintf(s, "Click: building %d roof", hoverB + 1);
  else
    sprintf(s, "Building %d: out of reach", hoverB + 1);
  labelBox((l + r) / 2, t - 62, s, BLACK, col);
}

// slim banner at the very top (does not cover anchors); details go to the side bar
void drawBanner()
{
  if (goalReached)
  {
    int cx = VIEW_W / 2, w = 16 * 13; // "GOAL REACHED!" at text size 2
    int bg = COLOR(10, 70, 35);
    setfillstyle(SOLID_FILL, bg);
    bar(cx - w / 2 - 14, 2, cx + w / 2 + 14, 38);
    setcolor(GREEN);
    rectangle(cx - w / 2 - 14, 2, cx + w / 2 + 14, 38);
    setbkcolor(bg);
    setcolor(WHITE);
    settextstyle(DEFAULT_FONT, HORIZ_DIR, 2);
    outtextxy(cx - w / 2, 4, (char *)"GOAL REACHED!");
    settextstyle(DEFAULT_FONT, HORIZ_DIR, 1);
    setbkcolor(BLACK);
  }
  else if (mode == M_CRASHED && crashT <= 0)
  {
    const char *s = "CRASHED - PRESS R";
    int w = 8 * (int)strlen(s), cx = VIEW_W / 2, bg = COLOR(120, 15, 20);
    setfillstyle(SOLID_FILL, bg);
    bar(cx - w / 2 - 10, 6, cx + w / 2 + 10, 32);
    setcolor(WHITE);
    rectangle(cx - w / 2 - 10, 6, cx + w / 2 + 10, 32);
    setbkcolor(bg);
    outtextxy(cx - w / 2, 11, (char *)s);
    setbkcolor(BLACK);
  }
}

void drawScene()
{
  drawSky();
  drawSkyline();
  drawClouds();
  drawBirds();
  drawGround();
  drawBuildings();
  // drawWebStrands();
  drawStreetLights();
  drawMarvelBanner();
  drawCars();
  drawStartGoal();
  drawGhost(); // planned route: above buildings/background, below the character
  drawTrail();
  drawAnchors();
  drawRope();
  drawCharacter();
  drawFx();
  drawCelebration();
  drawCrashBurst();
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

// [NEW] word-wrap long messages to the 34-character side bar
void hudWrap(int x, int &y, const char *s, int fg = WHITE)
{
  char ln[40];
  int n = (int)strlen(s), i = 0;
  while (i < n)
  {
    int len = n - i;
    if (len > 34)
      len = 34;
    if (i + len < n)
    {
      int k = len;
      while (k > 0 && s[i + k] != ' ')
        k--;
      if (k > 0)
        len = k;
    }
    memcpy(ln, s + i, len);
    ln[len] = 0;
    hud(x, y, ln, fg);
    i += len;
    while (i < n && s[i] == ' ')
      i++;
  }
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
  hudWrap(x, y, b);

  y += 4;
  hud(x, y, "-- GOAL & SEARCH --", LIGHTCYAN);
  sprintf(b, "Goal: bldg %d roof, x=%.1f m", goalBuilding + 1, goalPoint.x);
  hud(x, y, b);
  hudWrap(x, y, searchMsg);
  if (hasBest)
    sprintf(b, "Planned time: %.2f s", bestTime);
  else
    sprintf(b, "Planned time: --");
  hud(x, y, b);
  if (hasBest)
    sprintf(b, "Swings: %d | Flights tested: %ld", bestCount, nodes);
  else
    sprintf(b, "Swings: -- | Flights tested: %ld", nodes);
  hud(x, y, b);
  sprintf(b, "Route: %s", hasBest ? routeText : "--");
  hudWrap(x, y, b);
  sprintf(b, "Route preview: %s [V]", showPaths ? "ON" : "OFF");
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
  if (goalReached) // [NEW] result panel (replaces the legend while the success state is shown)
  {
    hud(x, y, "-- RESULT --", LIGHTGREEN);
    sprintf(b, "Actual time: %.2f s", elapsed);
    hud(x, y, b);
    if (hasBest)
      sprintf(b, "Planned time: %.2f s", bestTime);
    else
      sprintf(b, "Planned time: -- (manual)");
    hud(x, y, b);
    sprintf(b, "Swings used: %d", swingsUsed);
    hud(x, y, b);
    sprintf(b, "Anchors: %s", runRoute[0] ? runRoute : "--");
    hudWrap(x, y, b);
  }
  else
  {
    hud(x, y, "-- LEGEND --", LIGHTCYAN);
    legend(x, y, COLOR(255, 140, 0), "Actual trail", SOLID_LINE);
    legend(x, y, ROUTE_COL, "Planned route", DOTTED_LINE);
    legend(x, y, LIGHTRED, "Predicted landing", DOTTED_LINE);
    legend(x, y, COLOR(40, 200, 90), "Goal landing strip", SOLID_LINE);
  }
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
    strcpy(hint, "CRASHED - PRESS R to reset");
  else if (mode == M_SWING)
    strcpy(hint, "Swinging: SPACE releases now, or it auto-releases at the release angle");
  else if (mode == M_FLIGHT)
    strcpy(hint, "Flying: SPACE attaches to the selected anchor if it is in range");
  else if (goalReached)
    strcpy(hint, "Goal reached! R to try again, G to choose a new goal");
  else if (hasBest)
    strcpy(hint, "Route ready: V shows/hides it, X replays it, SPACE swings manually");
  else
    strcpy(hint, "SPACE attaches to the selected anchor, ENTER searches a route to the goal, G picks a goal");
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
  outtextxy(8, VIEW_H + 22, (char *)"G pick goal | V route on/off | P pause | N step | M slow-mo | D diagnostics | L level | T self-test | ESC exit");
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

// [FIXED] ENTER: always a fresh search toward the CURRENT goal strip, from the current state
void runSearch()
{
  if (goalReached)
  {
    strcpy(status, "Goal reached - press R first");
    return;
  }
  if (mode == M_CRASHED || mode == M_REPLAY)
    resetState(); // plan again from START
  Body s0 = body;
  bool inAir = (mode == M_FLIGHT || mode == M_SWING); // flying/swinging: plan from the current state
  int from = (mode == M_SWING) ? attachedAnchor : -1;
  strcpy(searchMsg, "Search: RUNNING...");
  updateHint();
  renderFrame(); // show the message before blocking
  clock_t c0 = clock();
  bool ok = findBestRoute(s0, inAir, from);
  double secs = (double)(clock() - c0) / CLOCKS_PER_SEC;
  if (ok)
  {
    showPaths = true; // route becomes visible immediately
    sprintf(searchMsg, "Route found in %.2f s%s", secs, searchAborted ? " (search limit hit)" : "");
    if (bestCount > 0)
    {
      selAnchor = bestSteps[0].anchor;
      releaseAngleDeg = bestSteps[0].angleDeg;
    }
    strcpy(status, "Route found - V toggles, X replays");
  }
  else
  {
    strcpy(searchMsg, "No route found within current search limits");
    strcpy(status, "No route found");
  }
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

  // goal landing-zone logic (restores the real goal afterwards)
  int oldB = goalBuilding, oldL = landedOn;
  Point2 oldG = goalPoint;
  double oldH = goalHalf;
  setGoalOnBuilding(1, 200.0); // far right of the roof -> must be clamped onto it
  check("goal clamped onto the roof", goalPoint.x >= buildings[1].x && goalPoint.x <= buildings[1].x + buildings[1].w);
  setGoalOnBuilding(4, 108.0);
  Body lb = {108.0, buildings[4].y + buildings[4].h + BODY_R, 0, 0};
  landedOn = 4;
  check("goal: landing inside strip accepted", atGoal(lb));
  lb.x = 100.0;
  check("goal: same roof, outside strip rejected", !atGoal(lb));
  lb.x = 108.0;
  landedOn = 0;
  check("goal: landing on another roof rejected", !atGoal(lb));
  landedOn = -1;
  check("goal: ground landing rejected", !atGoal(lb));
  goalBuilding = oldB;
  goalPoint = oldG;
  goalHalf = oldH;
  landedOn = oldL;
  printf("===== END =====\n\n");
}

// ===== STAGE 2/9: main loop =====
int main()
{
  initwindow(SCREEN_W, SCREEN_H, (char *)"HERE COMES SPIDEYYYY!!!", 100, 100, true, false);

  srand((unsigned)time(NULL));
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
    if (crashT > 0)
      crashT -= ft;
    if (celebT >= 0)
      celebT += ft;

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
    if (keyPressed['V']) // [FIXED] toggles the planned-route preview
    {
      showPaths = !showPaths;
      if (showPaths && !hasBest && mode == M_STAND)
        strcpy(status, "No planned route yet - press ENTER");
    }
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
      if (mode == M_STAND && !goalReached)
        strcpy(status, "READY - SPACE to attach");
    }
    if (keyPressed[VK_RIGHT])
    {
      selAnchor = (selAnchor + 1) % NUM_ANCHORS;
      if (mode == M_STAND && !goalReached)
        strcpy(status, "READY - SPACE to attach");
    }
    if (keyPressed[VK_UP])
    {
      releaseAngleDeg += 5;
      if (releaseAngleDeg > 70)
        releaseAngleDeg = 70;
    }
    if (keyPressed[VK_DOWN])
    {
      releaseAngleDeg -= 5;
      if (releaseAngleDeg < -70)
        releaseAngleDeg = -70;
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
      runSearch();
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