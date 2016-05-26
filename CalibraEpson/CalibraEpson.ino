//#########################################################################
//NEMA17
//totalSteps = 28602~
//#########################################################################
#define STEPSPERMILLIMETER = 108.7238095238095


int speedMillis = 2;
int steps = 8;
int direccion = 9;
int reset = 10;
int buttonA = 6;
int buttonB = 7;
int buttonAState = 0;
int buttonBState = 0;
bool forwarding = true;
int totalSteps = 0;
int currentStep = 0;

bool calibrating = false;
bool doingJob = false;
bool countingSteps = false;
bool goingHome = false;
long lastStep;
//#########################################################################
void setup() {
  // initialize the digital pin as an output.
  pinMode(steps, OUTPUT);
  pinMode(direccion, OUTPUT);
  pinMode(reset, OUTPUT);
  //digitalWrite(direccion, HIGH); forwarding = true;
  //digitalWrite(reset, HIGH);   //Cuando reset se encuentre en HIGH el motor arrancará y leerá los comandos enviados.

  pinMode(buttonA, INPUT);
  pinMode(buttonB, INPUT);

  Serial.begin(9600);
  Serial.setTimeout(100); //100 milis until close

  initializeMotors();
}
//#########################################################################
void loop() {

  //check boundaries
  checkLimits();

  //state
  if (calibrating || goingHome || doingJob)
  {
    
    //if (!forwarding && currentStep >= 142.2985074626866 * 50){ calibrating=false;return;}    
    doStep();

    if (goingHome && currentStep >= (totalSteps / 2)) {
      goingHome = false; //stop
      digitalWrite(reset, LOW);
      Serial.println("Home. Full Stop.");
    }
  }

  delay(1);//ms
}
//#########################################################################
//void readCommand()
//{
//  if (Serial.available() > 0) // Check to see if there is a new message
//  {
//    String message;
//    message = Serial.readString(); // Put the serial input into the message
//  }  
//}
//#########################################################################
void doStep()
{
  if (millis() - lastStep >= speedMillis) {
    digitalWrite(reset, HIGH);
    digitalWrite(steps, HIGH);  // This LOW to HIGH change is what creates the
    digitalWrite(steps, LOW); // al A4988 de avanzar una vez por cada pulso de energia.
    currentStep++;
    lastStep = millis();
  }
}
//#########################################################################
void initializeMotors()
{
  calibrating = true;
  goingHome = false;
  doingJob = false;
  countingSteps = false;
  digitalWrite(direccion, HIGH); forwarding = true;
  digitalWrite(reset, HIGH);
  totalSteps = 0;
  currentStep = 0;
  lastStep = millis();
}
//#########################################################################
void checkLimits()
{
  buttonAState = digitalRead(buttonA);
  buttonBState = digitalRead(buttonB);

  if (buttonAState == HIGH && buttonBState == HIGH)
  {
    initializeMotors();
    return;
  }

  //change direction
  if (forwarding && buttonBState == HIGH)
  {
    digitalWrite(direccion, LOW); forwarding = false;
    Serial.println("Stop. Reversing.");
    checkCalibration();
  }
  if (!forwarding && buttonAState == HIGH)
  {    
    digitalWrite(direccion, HIGH); forwarding = true;
    Serial.println("Stop. Forwarding.");
    checkCalibration();
  }
}
//#########################################################################
void checkCalibration()
{
  if (calibrating)
  {
    if (!countingSteps) {
      countingSteps = true;
      totalSteps = 0;
    }
    else
    {
      totalSteps = currentStep;
      calibrating = false;
      countingSteps = false;
      Serial.print("Stop. Total Length is "); Serial.println(totalSteps);

      goingHome = true;
      Serial.print("Going home.");
    }
  }
  currentStep = 0;
}




