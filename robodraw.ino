/*
 * robodraw.ino - the mighty robotic drawing arm
 *
 * Hail to the robots! Draw all humans!
 */

#include <Servo.h>

/****  Constants  ****/
/* Initial position in Cartesian coordinates (mm) */
const int INIT_X =  0;
const int INIT_Y = 20;
/* Movement step (mm) */
const int STEP = 8;
/* Movement delay (ms) */
const unsigned long DELAY_MOVE = 100;
/* Arms length (mm) */
const unsigned L1 = 110;
const unsigned L2 = 100;
/* Serial port data rate (bps) */
const long SERIAL_SPEED = 9600;
/* Shield pins */
const unsigned PIN_SERVO1 = 10;
const unsigned PIN_SERVO2 =  9;
#ifdef USE_BEEP
const unsigned PIN_BEEP = 7;
/* Tone frequences (Hz) */
const unsigned int  BEEP_OK_FREQ   = 440; 
const unsigned int  BEEP_FAIL_FREQ = 200;
/* Tone durations (ms) */
const unsigned long BEEP_OK_LEN    = 100;
const unsigned long BEEP_FAIL_LEN  = 100;
#endif
/* For use in coordinates conversion */
const float Pi  = 3.14159265;  /* √-1 8 ∑ π */
const float DEG = 180. / Pi;   /* radians to degrees conversion */
/* Some optimization */
const float L11 = (float)(L1 * L1);
const float L12 = (float)(L1 * L2) * 2.;
const float L22 = (float)(L2 * L2);

/****  Global variables  ****/
Servo Servo1, Servo2;  /* servos */
int X, Y;              /* current position */

/****  Initialization  ****/
void setup() {
  /* Begin COMmunication */
  Serial.begin(SERIAL_SPEED);

  /* Attach servos to pins */
  Servo1.attach(PIN_SERVO1);
  Servo2.attach(PIN_SERVO2);

  /* Move to initial position */
  move_to(INIT_X, INIT_Y);
}

/****  Main loop  ****/
void loop() {
  char cmd;

  if (Serial.available() > 0) {
    cmd = Serial.read();
    switch (cmd) {
      case '0': move_to(INIT_X, INIT_Y); break;  /* reset */

      case '2': line_v(-STEP); break;  /* North */
      case '4': line_h( STEP); break;  /* East  */
      case '6': line_v( STEP); break;  /* South */
      case '8': line_h(-STEP); break;  /* West  */

      case '1': line(-STEP, -STEP); break;  /* NW */
      case '3': line( STEP, -STEP); break;  /* NE */
      case '5': line( STEP,  STEP); break;  /* SE */
      case '7': line(-STEP,  STEP); break;  /* SW */
    }
  }
}

/****  Subroutines  ****/
#ifdef USE_BEEP
# define BEEP_OK beep_ok()
# define BEEP_FAIL beep_fail()

void beep_ok() {
  tone(PIN_BEEP, BEEP_OK_FREQ, BEEP_OK_LEN);
}

void beep_fail() {
  tone(PIN_BEEP, BEEP_FAIL_FREQ, BEEP_FAIL_LEN);
}
#else
# define BEEP_OK
# define BEEP_FAIL
#endif

/*
 * Move the arm directly to the specified point.
 *
 * Arguments: x, y - Cartesian coordinates (absolute)
 */
void move_to(int x, int y) {
  /* The arm can operate in the right half-plane only */
  if (y < 0) y = 0;

  float r = (float)x, r2 = (float)y;
  r2 = r * r + r2 * r2;  /* squared distance */
  r  = sqrt(r2);         /* distance */

  if (r > (float)(L1 + L2 - STEP)) {
    /* The arm cannot reach the specified point */
    BEEP_FAIL;
  } else {
    /* Convert Cartesian coordinates to servos angles */
    float c1 = (float)x / r;
    float c2 = (L11 - L22 + r2) / (2. * (float)L1 * r);
    float c3 = (L11 + L22 - r2) / L12;
    float a1 = 180. - (acos(c1) - acos(c2)) * DEG;
    float a2 = 180. - (Pi - acos(c3)) * DEG;

#ifdef DEBUG
    /* Debugging output to serial port */
    Serial.print(x);
    Serial.print("; ");
    Serial.print(y);
    Serial.print(" -> ");
    Serial.print(a1);
    Serial.print("; ");
    Serial.print(a2);
    Serial.println();
#endif

    /* Set servos angles */
    Servo1.write(a1);
    Servo2.write(a2);

    /* Save current position */
    X = x; Y = y;
  }

  /* Stay awhile (and listen) */
  delay(DELAY_MOVE);
}

/*
 * Move the arm relative to current position.
 *
 * Arguments: dx, dy - end point coordinates (relative)
 */
void move(int dx, int dy) {
  move_to(X + dx, Y + dy);
}

/*
 * Draw a horisontal line from current position.
 *
 * Arguments: x - end point X coordinate (relative)
 */
void line_h(int dx) {
  int s = dx > 0 ? 1 : -1;
  int x0 = X + s, x1 = X + dx;

  for (int i = x0; i != x1; i += s) {
    move_to(i, Y);
  }
}

/*
 * Draw a vertical line from current position.
 *
 * Arguments: y - end point Y coordinate (relative)
 */
void line_v(int dy) {
  int s = dy > 0 ? 1 : -1;
  int y0 = Y + s, y1 = Y + dy;

  for (int i = y0; i != y1; i += s) {
    move_to(X, i);
  }
}

/*
 * Draw a line from current position to the specified point
 * using Bresenheim algorithm.
 *
 * Arguments: x, y - end point coordinates (relative)
 */
void line(int x, int y) {
  int x1 = X + x, dx = abs(x), sx = x > 0 ? 1 : -1;
  int y1 = Y + y, dy = abs(y), sy = y > 0 ? 1 : -1;
  int e1 = (dx > dy ? dx : -dy) / 2, e2;
  x = X; y = Y;

  for (;;) {
    move_to(x, y);
    if (x == x1 && y == y1) break;
    e2 = e1;
    if (e2 > -dx) { e1 -= dy; x += sx; }
    if (e2 <  dy) { e1 += dx; y += sy; }
  }
}

