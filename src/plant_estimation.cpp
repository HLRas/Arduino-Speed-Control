#include <Arduino.h>

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
#define GAPS 30.0
#define MEAS_TIME_INC 200.0

// Plant estimation
bool estimate = false;
unsigned long start_time = 0;
unsigned long step_start_time = 0;
unsigned long step_duration = 10000; // 10 seconds in milliseconds
bool step_applied = false;
float step_input_pwm = 200;

// pwm inputs
int pwmL = 0;
int pwmR = 0;

// PID controller
float desiredSpeedL = 1.5; // m/s
float desiredSpeedR = 1.5; // m/s

// PID terms
const float KpL = 12, KiL = 0.18, KdL = 0;
const float KpR = 13, KiR = 0.28, KdR = 0;
float errorL = 0, errorR = 0;
float derivL = 0, derivR = 0;
float integralL = 0, integralR = 0;
float prevErrorL = 0, prevErrorR = 0;

void setup(){
    Serial.begin(9600); // must match Jetson's baud rate
    //while (!Serial);    // wait for serial to be ready (needed for some boards)
    Serial.println("Time,PWM_Left,PWM_Right,LeftSpeed,RightSpeed,StepApplied");
    
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
    attachInterrupt(digitalPinToInterrupt(speedPinR), addCountR, RISING);
    attachInterrupt(digitalPinToInterrupt(speedPinL), addCountL, RISING);

    // Apply forward moving wheels
    forwardL();
    forwardR();
}

void loop(){
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
            
            // Limit PWM range
            pwmL = constrain(pwmL, 0, 255);
            pwmR = constrain(pwmR, 0, 255);

            // Apply PWM
            analogWrite(enAR, pwmR); // Right motor
            analogWrite(enBL, pwmL); // Left motor

            //printSpeeds();
        }

    }
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

void printSpeeds(){
    Serial.print(currentTime);
    Serial.print(",");
    Serial.print(speedL);
    Serial.print(",");
    Serial.println(speedR);
}

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

// Increment speed counts
void addCountR() { countR++; }
void addCountL() { countL++; }

// Motor direction functions
void forwardL() { digitalWrite(in1R, LOW); digitalWrite(in2R, HIGH); }
void reverseL() { digitalWrite(in1R, HIGH);  digitalWrite(in2R, LOW); }
void forwardR() { digitalWrite(in4L, LOW); digitalWrite(in3L, HIGH); }
void reverseR() { digitalWrite(in4L, HIGH);  digitalWrite(in3L, LOW); }