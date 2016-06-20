//------------------------------------------------------------------------------
// 3 (daniel) Axis CNC Demo  - supports CNCShieldV3 on arduino UNO
// dan@marginallyclever.com 2013-10-28
// Modified by Søren Vedel
// sorenvedel@gmail.com 2015-06-19
// Modified by Cristian Leiva
// cristianleiva@gmail.com 2016-04-28
// add Support cncshield - feed rate in mm/min

//------------------------------------------------------------------------------
// Copyright at end of file.
// please see http://www.github.com/MarginallyClever/GcodeCNCDemo for more information.


//------------------------------------------------------------------------------
// CONSTANTS
//------------------------------------------------------------------------------
//#define VERBOSE              (1)  // add to get a lot more serial output.
#include <Servo.h>

#define VERSION              (2)  // firmware version
#define BAUD                 (115200)  // How fast is the Arduino talking?
#define MAX_BUF              (128)  // What is the longest message Arduino can store?
#define STEPS_PER_TURN       (200)  // depends on your stepper motor.  most are 200.
#define STEPS_PER_MMX        (142.2985074626866)  // EM-483 nema17 varilla roscada 8mm
#define STEPS_PER_MMY        (142.2985074626866) // EM-258 nema17 varilla roscada 8mm
#define STEPS_PER_MMZ        (6) // microservo 6 degrees = 1mm
#define MAX_FEEDRATE         (500) //500 steps per second
#define MIN_FEEDRATE         (1) //1 steps per second
#define NUM_AXIES            (2)
#define BACK_STEPS_X           (70)//steps to do when changing direction
#define BACK_STEPS_Y           (35)//steps to do when changing direction

// for arc directions
#define ARC_CW          (1)
#define ARC_CCW         (-1)
// Arcs are split into many line segments.  How long are the segments?
#define MM_PER_SEGMENT  (10)

#define JOY_PIN_X             (0)
#define JOY_PIN_Y             (1)
#define JOY_PIN_SWITCH        (2)
#define JOY_DEADZONE_LOW      (460) //10%
#define JOY_DEADZONE_HIGH     (563) //10%

//------------------------------------------------------------------------------
// STRUCTS
//------------------------------------------------------------------------------
// for line()
typedef struct {
  long delta;  // number of steps to move
  long absdelta;
  long over;  // for dx/dy bresenham calculations
} Axis;


typedef struct {
  int step_pin;
  int dir_pin;
  int enable_pin;
  int limit_switch_pin;
  int last_dir;
} Motor;


//------------------------------------------------------------------------------
// GLOBALS
//------------------------------------------------------------------------------
Axis a[NUM_AXIES];  // for line()
Axis atemp;  // for line()
Motor motors[NUM_AXIES];
Servo servo[2];

char buffer[MAX_BUF];  // where we store the message until we get a ';'
int sofar;  // how much is in the buffer

// speeds
float fr = 0; // human version
long step_delay;  // machine version

float px, py, pz; // position

// settings
char mode_abs = 1; // absolute mode?

long line_number = 0;

int motorDelay = 2000; //microSeconds

boolean newData = false;

boolean lastJoySwitchState = false;
//------------------------------------------------------------------------------
// METHODS
//------------------------------------------------------------------------------


/**
   delay for the appropriate number of microseconds
   @input ms how many milliseconds to wait
*/
void pause(long ms) {

  if (ms < 2) ms = 2;//motor limit

  delay(ms);// / 1000);
  delayMicroseconds((ms * 1000) % 1000); // delayMicroseconds doesn't work for values > ~16k.
}
/**
   Set the feedrate (speed motors will move)
   @input nfr the new speed in steps/second
*/
void feedrate(float nfr) {
  nfr = nfr * STEPS_PER_MMX / 60;
  if (fr == nfr) return; // same as last time?  quit now.

  if (nfr > MAX_FEEDRATE) nfr = MAX_FEEDRATE;
  if (nfr < MIN_FEEDRATE) nfr = MIN_FEEDRATE;

  step_delay = MAX_FEEDRATE / nfr;
  fr = nfr;
}


/**
   Set the logical position
   @input npx new position x
   @input npy new position y
*/
void position(float npx, float npy, float npz) {
  // here is a good place to add sanity tests
  px = npx;
  py = npy;
  pz = npz;
}


/**
   Supports movement with both styles of Motor Shield
   @input newx the destination x position
   @input newy the destination y position
 **/
void onestep(int motor) {
#ifdef VERBOSE
  char *letter = "XYZE";
  Serial.print(letter[]);
#endif

  digitalWrite(motors[motor].step_pin, HIGH);
  digitalWrite(motors[motor].step_pin, LOW);
}

//check if safety buttons are pressed
bool checkBoundaries()
{
  for (int i = 0; i < NUM_AXIES; ++i) {
    if (digitalRead(motors[i].limit_switch_pin) == HIGH) return true;
  }
  return false;
}


/**
   Uses bresenham's line algorithm to move both motors
   @input newx the destination x position
   @input newy the destination y position
 **/
void line(float newx, float newy, float newz) {

  //motor_enable();

  a[0].delta = (newx - px) * STEPS_PER_MMX;
  a[1].delta = (newy - py) * STEPS_PER_MMY;

  //SERVO////////////////////////////////////
  //10mm = 180, -10mm = 0
  //if (newz > 10.0) newz = 10.0;
  //if (newz < -10.0) newz = -10.0;
  if (pz != newz)
  {
    int dgr = (int)((newz / 10.0 * 90.0) + 90);
    servo[0].attach(11);
    servo[0].write(dgr);
    pause(300);
    servo[0].detach();
    //servo[1].write(180 - dgr);
    pz = newz;
  }
  ////////////////////////////////////

  long i, j, k, maxsteps = 0;

  for (i = 0; i < NUM_AXIES; ++i) {
    a[i].absdelta = abs(a[i].delta);
    a[i].over = 0;
    if ( maxsteps < a[i].absdelta ) maxsteps = a[i].absdelta;

    // set the direction once per movement
    boolean forward = (a[i].delta > 0);
    digitalWrite(motors[i].dir_pin, forward ? HIGH : LOW);

    //bounce compensation
    if (motors[i].last_dir != (forward ? 1 : 0))
    {
      if ((i == 0 && px != newx) || (i == 1 && py != newy))
      {
        Serial.print("compensating bound for ");
        Serial.println(i == 0 ? "x" : "y");
        for (k = 0; k < (i == 0 ? BACK_STEPS_X : BACK_STEPS_Y); k++) {
          onestep(i);
          pause(motorDelay / 1000);
        }
        motors[i].last_dir = (forward ? 1 : 0);
      }
    }
  }

  for ( i = 0; i < maxsteps; ++i ) {

    if (checkBoundaries()) {
      break;
    }

    for (j = 0; j < NUM_AXIES; ++j) {
      a[j].over += a[j].absdelta;
      if (a[j].over >= maxsteps) {
        a[j].over -= maxsteps;
        onestep(j);
      }
    }

    pause(motorDelay / 1000);
  }

  position(newx, newy, newz);

  where();
}


// returns angle of dy/dx as a value from 0...2PI
static float atan3(float dy, float dx) {
  float a = atan2(dy, dx);
  if (a < 0) a = (PI * 2.0) + a;
  return a;
}


// This method assumes the limits have already been checked.
// This method assumes the start and end radius match.
// This method assumes arcs are not >180 degrees (PI radians)
// cx/cy - center of circle
// x/y - end position
// dir - ARC_CW or ARC_CCW to control direction of arc
void arc(float cx, float cy, float x, float y, float dir) {

  // get radius
  float dx = px - cx;
  float dy = py - cy;
  float radius = sqrt(dx * dx + dy * dy);

  // find angle of arc (sweep)
  float angle1 = atan3(dy, dx);
  float angle2 = atan3(y - cy, x - cx);
  float theta = angle2 - angle1;

  if (dir > 0 && theta < 0) angle2 += 2 * PI;
  else if (dir < 0 && theta > 0) angle1 += 2 * PI;

  theta = angle2 - angle1;

  // get length of arc
  // float circ=PI*2.0*radius;
  // float len=theta*circ/(PI*2.0);
  // simplifies to
  float len = abs(theta) * radius;

  int i, segments = ceil( len * MM_PER_SEGMENT );

  float nx, ny, angle3, scale;

  for (i = 0; i < segments; ++i) {
    // interpolate around the arc

    //if ((float)segments == 0)Serial.println("Zero division found");

    scale = ((float)i) / ((float)segments);

    angle3 = ( theta * scale ) + angle1;
    nx = cx + cos(angle3) * radius;
    ny = cy + sin(angle3) * radius;
    // send it to the planner
    line(nx, ny, pz);
  }

  line(x, y, pz);
}


/**
   Look for character /code/ in the buffer and read the float that immediately follows it.
   @return the value found.  If nothing is found, /val/ is returned.
   @input code the character to look for.
   @input val the return value if /code/ is not found.
 **/
float parsenumber(char code, float val) {
  char *ptr = buffer;
  while (ptr && *ptr && ptr < buffer + sofar) {
    if (*ptr == code) {
      return atof(ptr + 1);
    }
    ptr = strchr(ptr, ' ') + 1;
  }
  return val;
}


/**
   write a string followed by a float to the serial line.  Convenient for debugging.
   @input code the string.
   @input val the float.
*/
void output(const char *code, float val) {
  Serial.print(code);
  Serial.print(val);
  Serial.print(" ");
}


void setDelay(int _delay)
{
  if (_delay < 2000) _delay = 2000;
  motorDelay = _delay;
}

/**
   print the current position, feedrate, and absolute mode.
*/
void where() {
  output("X", px);
  output("Y", py);
  output("Z", pz);
  output("F", fr / STEPS_PER_MMX * 60);
  Serial.println(mode_abs ? "ABS" : "REL");
}


/**
   display helpful information
*/
void help() {
  Serial.println(VERSION);
  Serial.println(F("Commands:"));
  Serial.println(F("G00 [X(steps)] [Y(steps)] [F(feedrate)]; - line"));
  Serial.println(F("G01 [X(steps)] [Y(steps)] [F(feedrate)]; - line"));
  Serial.println(F("G02 [X(steps)] [Y(steps)] [I(steps)] [J(steps)] [F(feedrate)]; - clockwise arc"));
  Serial.println(F("G03 [X(steps)] [Y(steps)] [I(steps)] [J(steps)] [F(feedrate)]; - counter-clockwise arc"));
  Serial.println(F("G04 P[seconds]; - delay"));
  Serial.println(F("G90; - absolute mode"));
  Serial.println(F("G91; - relative mode"));
  Serial.println(F("G92 [X(steps)] [Y(steps)]; - change logical position"));
  Serial.println(F("G99 Calibrate to Center"));
  Serial.println(F("M18; - disable motors"));
  Serial.println(F("M100; - this help message"));
  Serial.println(F("M112; - EMERGENCY STOP"));
  Serial.println(F("M114; - report position and feedrate"));
  Serial.println(F("M800 D(delay); -set delay in microseconds"));
  Serial.println(F("All commands must end with a newline."));
}

void processCommand() {

  int cmd = parsenumber('G', -1);
  switch (cmd) {
    case  0:
    case  1: { // line
        feedrate(parsenumber('F', fr));
        line( parsenumber('X', (mode_abs ? px : 0)) + (mode_abs ? 0 : px),
              parsenumber('Y', (mode_abs ? py : 0)) + (mode_abs ? 0 : py),
              parsenumber('Z', (mode_abs ? pz : 0)) + (mode_abs ? 0 : pz)
            );
        break;
      }
    case 2:
    case 3: {  // arc
        feedrate(parsenumber('F', fr));
        arc(parsenumber('I', 0) + px,
            parsenumber('J', 0) + py,
            parsenumber('X', (mode_abs ? px : 0)) + (mode_abs ? 0 : px),
            parsenumber('Y', (mode_abs ? py : 0)) + (mode_abs ? 0 : py),
            (cmd == 2) ? -1 : 1);
        break;
      }
    case  4:  pause(parsenumber('P', 0) * 1000);  break; // dwell
    case 90:  mode_abs = 1;  break; // absolute mode
    case 91:  mode_abs = 0;  break; // relative mode
    case 92:  // set logical position
      position( parsenumber('X', 0),
                parsenumber('Y', 0),
                parsenumber('Z', 0)
              );
      break;
    case 99:
      //calibrate();
      break;
    default:  break;
  }

  cmd = parsenumber('M', -1);
  switch (cmd) {
    case 112:
    case 18:  // disable motors
      motor_disable();
      break;
    case 100:  help();  break;
    case 114:  where();  break;
    case 800:  setDelay(parsenumber('D', 2000));  break;
    default:  break;
  }
}

/**
   prepares the input buffer to receive a new message and tells the serial connected device it is ready for more.
*/
void ready() {
  sofar = 0; // clear input buffer
  Serial.print(F(">"));  // signal ready to receive input
}


/**
   set up the pins for each motor
   Pins fits a CNCshieldV3.xx
*/
void motor_setup() {
  //axis X - BED
  motors[0].step_pin = 3;
  motors[0].dir_pin = 4;
  motors[0].enable_pin = 5;
  motors[0].limit_switch_pin = 7;
  motors[0].last_dir = 0;

  //axis Y - TOWER
  motors[1].step_pin = 8;
  motors[1].dir_pin = 9;
  motors[1].enable_pin = 10;
  motors[1].limit_switch_pin = 6;
  motors[1].last_dir = 0;

  //servo
  //servo[0].attach(11);
  //servo[1].attach(12);

  int i;
  for (i = 0; i < NUM_AXIES; ++i) {
    // set the motor pin & scale
    pinMode(motors[i].step_pin, OUTPUT);
    pinMode(motors[i].dir_pin, OUTPUT);
    pinMode(motors[i].enable_pin, OUTPUT);
  }
}


void motor_enable() {
  int i;
  for (i = 0; i < NUM_AXIES; ++i) {
    digitalWrite(motors[i].enable_pin, HIGH);
  }
  //servo[0].attach(11);
  //servo[1].attach(12);
}


void motor_disable() {
  int i;
  for (i = 0; i < NUM_AXIES; ++i) {
    digitalWrite(motors[i].enable_pin, LOW);
  }
  /*for (i = 0; i < 2; ++i) {
    servo[i].detach();
    }*/

}

/**
   First thing this machine does on startup.  Runs only once.
*/
void setup() {
  Serial.begin(BAUD);  // open coms

  motor_setup();

  motor_enable();

  where();  // for debugging purposes
  help();  // say hello
  position(0, 0, 0); // set starting position
  feedrate(100);  // set default speed


  //servo[0].write(90);
  //servo[1].write(90);

  //calibrate();

  ready();

}


//---------------------------------------------------------
void loop() {

  // listen for serial commands
  recvWithEndMarker();

  if (newData == true) {
    processCommand();
    ready();
    Serial.print("ok\r\n");
    newData = false;
  }

  processJoy();
}
//---------------------------------------------------------
void processJoy()
{
  int steps = 0;

  //switch
  int joySwitch = analogRead(JOY_PIN_SWITCH);
  if (joySwitch == 0) {
    Serial.println(joySwitch);
    servo[0].attach(11);
    servo[0].write(lastJoySwitchState ? 135 : 81);
    pause(300);
    servo[0].detach();
    lastJoySwitchState = !lastJoySwitchState;
  }

  //int joyPause = (int)(8.0 / 512.0 * abs(joyX-512)) + 2;  //0-512 -> 2ms - 10ms
  int joyX = analogRead(JOY_PIN_X);
  if (joyX < JOY_DEADZONE_LOW) {
    digitalWrite(motors[0].dir_pin, HIGH);
    onestep(0);
    steps++;
  }
  if (joyX > JOY_DEADZONE_HIGH) {
    digitalWrite(motors[0].dir_pin, LOW);
    onestep(0);
    steps++;
  }
  int joyY = analogRead(JOY_PIN_Y);
  if (joyY < JOY_DEADZONE_LOW) {
    digitalWrite(motors[1].dir_pin, HIGH);
    onestep(1);
    steps++;
  }
  if (joyY > JOY_DEADZONE_HIGH) {
    digitalWrite(motors[1].dir_pin, LOW);
    onestep(1);
    steps++;
  }

  if (steps > 0) delay(motorDelay / 1000);
  
  //Serial.print(joyX);
  //Serial.print(",");
  //Serial.println(joyY);

}
//---------------------------------------------------------
void recvWithEndMarker() {
  static byte ndx = 0;
  char endMarker = '\n';
  char rc;
  sofar = 0;
  // if (Serial.available() > 0) {
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();

    if (rc != endMarker) {
      buffer[ndx] = rc;
      ndx++;
      if (ndx >= MAX_BUF) {
        ndx = MAX_BUF - 1;
      }
    }
    else {
      buffer[ndx] = '\0'; // terminate the string
      sofar = ndx;
      ndx = 0;
      newData = true;
    }
  }
}
