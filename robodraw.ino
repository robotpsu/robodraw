/*
 * robodraw.ino - the mighty robotic drawing arm
 *
 * Hail to the robots! Draw all humans!
 */

#include <Servo.h>

/****  Type definitions  ****/
/* A point in Cartesian coordinates */
typedef struct {
  int x, y;
} Point;

/****  Constants  ****/
const int L1 = 110;            /* arms length in mm */
const int L2 = 110;
const int MoveDelay = 100;     /* movement delay in ms */
/* For use in coordinates conversion */
const float Pi  = 3.14159265;  /* √-1 8 ∑ π */
const float Deg = 180. / Pi;   /* radians to degrees conversion */
/* Some optimization */
const int L11 = L1 * L1;
const int L12 = L1 * L2 * 2;
const int L22 = L2 * L2;

/****  Global variables  ****/
Servo Servo1, Servo2;  /* servos */
float X, Y;            /* current position */

/****  Initialization  ****/
void setup() {
  /* Begin COMmunication */
  Serial.begin(9600);

  /* Attach servos to pins */
  Servo1.attach(10);
  Servo2.attach(9);

  /* Move to initial position */
  move_to(0, 130);
}

/****  Main loop  ****/
void loop() {
  const int SquareSize = 20;
  const int SquareNum  =  4;

  for (int i = 0; i < SquareNum; i++) {
    rect(SquareSize, SquareSize);
    if (i < SquareNum - 1) line_v(SquareSize * 2);
  }

  exit(0);
}

/****  Subroutines  ****/

/*
 * Move the arm to the specified point.
 *
 * Arguments: x, y - Cartesian coordinates (absolute)
 */
void move_to(int x, int y) {
  float r, r2, a1, a2;

  r2 = x * x + y * y;  /* squared distance */
  r  = sqrt(r2);       /* distance */

  if (r > L1 + L2 - 5) {
    /* Safely fold the arm */
    a1 = 180.; a2 = 20.;
    x = 0; y = L1 + L2;
  } else {
    /* Convert Cartesian coordinates to servos angles */
    a1 = 180. - (acos(x / r) - acos((L11 - L22 + r2) / (2 * L1 * r))) * Deg;
    a2 = 180. - (Pi - acos((L11 + L22 - r2) / L12)) * Deg;
  };

  /* Set servos angles */
  Servo1.write(a1);
  Servo2.write(a2);

  /* Save current position */
  X = x; Y = y;

  /* Stay awhile (and listen) */
  delay(MoveDelay);
}

/*
 * Move the arm relative to current position.
 *
 * Arguments: dx, dy - coordinate deltas
 */
void move(float dx, float dy) {
  move_to(X + dx, Y + dy);
}

/*
 * Draw a horisontal line from current position.
 *
 * Arguments: x - line length and direction (sign)
 */
void line_h(int dx) {
  int d = dx > 0 ? 1 : -1;
  int x0 = X + d, x1 = X + dx;

  for (int i = x0; i != x1; i += d) {
    move_to(i, Y);
  }
}

/*
 * Draw a vertical line from current position.
 *
 * Arguments: y - line length and direction (sign)
 */
void line_v(int dy) {
  int d = dy > 0 ? 1 : -1;
  int y0 = Y + d, y1 = Y + dy;

  for (int i = y0; i != y1; i += d) {
    move_to(X, i);
  }
}

/*
 * Draw a line from current position to the specified point
 * using Bresenheim algorithm.
 *
 * Arguments: x, y - Cartesian coordinates (absolute)
 */
void line_to(int x, int y) {
  int dx = abs(x - X), sx = X < x ? 1 : -1;
  int dy = abs(y - Y), sy = Y < y ? 1 : -1;
  int e1 = (dx > dy ? dx : -dy) / 2, e2;

  while (X != x || Y != y) {
    e2 = e1;
    if (e2 > -dx) { e1 -= dy; X += sx; }
    if (e2 <  dy) { e1 += dx; Y += sy; }
    move_to(X, Y);
  }
}

/*
 * Draw a rectangle with sides parallel to axes.
 *
 * Arguments: dx, dy - rectangle size (coordinate deltas)
 */
void rect(int dx, int dy) {
  line_h( dx);
  line_v( dy);
  line_h(-dx);
  line_v(-dy);
}

/*
 * Draw a polygonal chain using array of points.
 *
 * Arguments: path[] - array of points
 */
void polygon(Point path[]) {
  for (int i = 0; i < sizeof(path); i++) {
    line_to(path[i].x, path[i].y);
  }
}

