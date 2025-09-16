#include <Arduino.h>

// Max speed for this car is roughly 0.5m/s on carpet

void plotSpeeds();
void plantEstimation();
void printPlantData();
void turnOff(int whichMotor = 2);
void forwardL();
void reverseL();
void forwardR();
void reverseR();
void addCountL();
void addCountR();
void printSpeeds();
void runComms();
void adjustDirection();
void applyPWMs(bool constrain_pwm = true);

// Reference to where motor is, look from back to front of car
// Right motor
const uint8_t enAR = 9;
const uint8_t in1R = 8;
const uint8_t in2R = 7;
const uint8_t speedPinR = 3;
volatile uint16_t countR = 0;
volatile float speedR = 0;

// Left motor
const uint8_t enBL = 11;
const uint8_t in3L = 12;
const uint8_t in4L = 13;
const uint8_t speedPinL = 2;
volatile uint16_t countL = 0;
volatile float speedL = 0;

// Timing variables
unsigned long currentTime = 0;
unsigned long prevTime = 0;
float timePassed = 0;

// Constants for measuring speed
#define GAPS 60.0  // Rising and falling edge detection
#define MEAS_TIME_INC 200.0
#define MIN_PWM 80 // This allows for a nicer response
#define MAX_PWM 255 // Max allowed pwm

// Plant estimation
bool estimate = false;
unsigned long start_time = 0;
unsigned long step_start_time = 0;
unsigned long step_duration = 10000; // 10 seconds in milliseconds
bool step_applied = false;
float step_input_pwm = 400;

// pwm inputs (small initial value to prevent integral windup)
int pwmL = 80;
int pwmR = 80;

// PID controller
float desiredSpeedL = 0.3; // m/s
float desiredSpeedR = 0.3; // m/s

// PID terms
const float KpL = 10, KiL = 0, KdL = 0; // 200ms ts
const float KpR = 10, KiR = 0, KdR = 0;

//const float KpL = 10, KiL = 0, KdL = 0; // 100ms ts
//const float KpR = 10, KiR = 0, KdR = 0;

float errorL = 0, errorR = 0;
float derivL = 0, derivR = 0;
float integralL = 0, integralR = 0;
float prevErrorL = 0, prevErrorR = 0;

// Comms
bool echoSpeeds = false ;

// Straight pwm input for testing
bool straight = false ;

void setup(){
    Serial.begin(9600); // must match Jetson's baud rate
    while (!Serial);    // wait for serial to be ready (needed for some boards)
    //Serial.println("Time,PWM_Left,PWM_Right,LeftSpeed,RightSpeed,StepApplied");
    
    // Initialize timing
    start_time = millis();
    step_start_time = millis();

    // Set up motor outputs
    pinMode(enAR, OUTPUT);
    pinMode(enBL, OUTPUT);
    pinMode(in1R, OUTPUT);
    pinMode(in2R, OUTPUT);
    pinMode(in3L, OUTPUT);
    pinMode(in4L, OUTPUT);

    // Set up speed sensor inputs
    pinMode(speedPinL, INPUT);
    pinMode(speedPinR, INPUT);

    // Turn off motors - Initial state
    turnOff();

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(speedPinR), addCountR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(speedPinL), addCountL, CHANGE);

    // Apply forward moving wheels
    forwardL();
    forwardR();

    // Apply a starting pwm
    applyPWMs();
}

void loop(){
    //runComms();
    adjustDirection();
    if (!straight){ // If not taking a straight pwm input
      currentTime = millis();

      timePassed = currentTime - prevTime;

      if (timePassed >= MEAS_TIME_INC){
          timePassed /= 1000.0;
          prevTime = currentTime ;
          speedL = countL / (GAPS * timePassed);
          speedR = countR / (GAPS * timePassed);
          countL = 0;
          countR = 0;

          // Plant estimation or PID control
          if (estimate){
              plantEstimation();
          }else{
              // PID Control mode
              // Left wheel PID
              errorL = desiredSpeedL - speedL;
            
              integralL += errorL * (timePassed);
              derivL = (errorL - prevErrorL) / (timePassed);
              pwmL += KpL * errorL + KiL * integralL + KdL * derivL;
              prevErrorL = errorL;

              // Right wheel PID
              errorR = desiredSpeedR - speedR;
              
              integralR += errorR * (timePassed);
              derivR = (errorR - prevErrorR) / (timePassed);
              pwmR += KpR * errorR + KiR * integralR + KdR * derivR;
              prevErrorR = errorR;              

              // Apply PWM
              applyPWMs();
              
              if (echoSpeeds){
                  Serial.print("ACK:");
                  Serial.print(desiredSpeedL, 3);
                  Serial.print(",");
                  Serial.println(desiredSpeedR, 3);
              }else{
                printSpeeds();
              }
              
          }

      }
    }else{ // apply straight pwm input without constraining pwm
      applyPWMs(false);
    }
    
}

// Change motor direction according to desired speed
void adjustDirection(){ // This still needs work!
    if (desiredSpeedL < 0){reverseL();} else {forwardL();};
    if (desiredSpeedR < 0){reverseR();} else {forwardR();};
}

// Turn off motor
void turnOff(int whichMotor) {
  if (whichMotor == 2 || whichMotor == 0) { // Left or Both
    analogWrite(enAR, 0);
    digitalWrite(in1R, LOW);
    digitalWrite(in2R, LOW);
  }
  if (whichMotor == 2 || whichMotor == 1) { // Right or Both
    analogWrite(enBL, 0);
    digitalWrite(in3L, LOW);
    digitalWrite(in4L, LOW);
  }
}

// Run communication protocol
void runComms(){
    if (Serial.available() > 0) {
    String msg = Serial.readStringUntil('\n');
    msg.trim(); // Remove any whitespace/newline characters
    
    // Find the comma delimiter
    int commaIndex = msg.indexOf(',');
    
    if (commaIndex > 0) {
      // Split the message into two parts
      String leftSpeedStr = msg.substring(0, commaIndex);
      String rightSpeedStr = msg.substring(commaIndex + 1);
      
      // Convert to float and assign to desired speeds
      desiredSpeedL = leftSpeedStr.toFloat()*2;
      desiredSpeedR = rightSpeedStr.toFloat()*2;
      
      // Optional: Send confirmation back to Jetson
      Serial.print("ACK:");
      Serial.print(desiredSpeedL, 3);
      Serial.print(",");
      Serial.println(desiredSpeedR, 3);
    }
  }
}

// Print the speeds (This is for the local terminal only!)
void printSpeeds(){
    Serial.print(currentTime);
    Serial.print(",");
    Serial.print(speedL);
    Serial.print(",");
    Serial.println(speedR);
}

// Estimating the plant (Didn't really use this)
void plantEstimation(){
  float elapsed_time = (currentTime - start_time) / 1000.0;
  
  // Apply step input after 2 seconds to allow system to settle
  if (elapsed_time >= 2.0 && !step_applied && elapsed_time < (2.0 + step_duration/1000.0)) {
    step_applied = true;
    pwmL = step_input_pwm;
    pwmR = step_input_pwm;
  }
  // Remove step input after step duration
  else if (elapsed_time >= (2.0 + step_duration/1000.0) && step_applied) {
    pwmL = 0;
    pwmR = 0;
    step_applied = false;
  }
  
  // Apply PWM to motors
  analogWrite(enAR, pwmR); // Right motor
  analogWrite(enBL, pwmL); // Left motor
  
  // Print data for MATLAB analysis
  printPlantData();
}

// Print plant estimation data in MATLAB-friendly format
void printPlantData(){
  float elapsed_time = (currentTime - start_time) / 1000.0;
  
  Serial.print(elapsed_time, 3); // Time
  Serial.print(",");
  Serial.print(pwmL); // PWM Left (input)
  Serial.print(",");
  Serial.print(pwmR); // PWM Right (input)
  Serial.print(",");
  Serial.print(speedL, 3); // Left speed (output)
  Serial.print(",");
  Serial.print(speedR, 3); // Right speed (output)
  Serial.print(",");
  Serial.println(step_applied ? 1 : 0); // Step applied flag
}

void applyPWMs(bool constrain_pwm){
  if (constrain_pwm){
    pwmL = constrain(pwmL, MIN_PWM, MAX_PWM);
    pwmR = constrain(pwmR, MIN_PWM, MAX_PWM);
  }
  analogWrite(enAR, pwmR); // Right motor
  analogWrite(enBL, pwmL); // Left motor
}

// Increment speed counts
void addCountR() { countR++; }
void addCountL() { countL++; }

// Motor direction functions
void forwardL() { digitalWrite(in1R, LOW); digitalWrite(in2R, HIGH); }
void reverseL() { digitalWrite(in1R, HIGH);  digitalWrite(in2R, LOW); }
void forwardR() { digitalWrite(in4L, LOW); digitalWrite(in3L, HIGH); }
void reverseR() { digitalWrite(in4L, HIGH);  digitalWrite(in3L, LOW); }