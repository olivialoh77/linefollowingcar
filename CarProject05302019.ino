// IR SENSOR

const int IRSensor0 = 65;
const int IRSensor1 = 48;
const int IRSensor2 = 64;
const int IRSensor3 = 47;
const int IRSensor4 = 52;
const int IRSensor5 = 68;
const int IRSensor6 = 53;
const int IRSensor7 = 69;

bool lineSensed = false;

const int power_odd = 45;
const int power_even = 61;

//IR PID Constants
float prevValue = 0;
//float prevTime = 0;
//float TimeDifference;
const float KP = 20;
//const float KI = 0;
const float KD = 0;

//ENCODER PID Constants
const int conversionConstant = 400;
const int KP2 = 30;

// MOTOR CONTROL
const int left_nslp_pin = 31; //digital write
const int left_dir_pin = 29; //digital write
const int left_pwm_pin = 40; //analog write

const int right_nslp_pin = 11;
const int right_dir_pin = 30;
const int right_pwm_pin = 39;

int velocityMax = 75;

//MANUEVERS

const int enterZone = 1;
const int leaveZone = 2;

int count = 0;

// ENCODER CONSTANTS
const float distanceMeasured = (10.0 / 144.0) * (2.0 * 3.1415 * 7.0);
float diffLA;
float diffRA;

//LED light 
const int LED_pin = 51;

void setup() {
  //Serial.begin(9600);

  // IR SENSOR
  pinMode(power_odd, OUTPUT);
  pinMode(power_even, OUTPUT);

  digitalWrite(power_odd, HIGH);
  digitalWrite(power_even, HIGH);

  // MOTOR CONTROL
  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin, OUTPUT);
  pinMode(left_pwm_pin, OUTPUT);

  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin, OUTPUT);
  pinMode(right_pwm_pin, OUTPUT);

  digitalWrite(left_nslp_pin, HIGH);
  digitalWrite(right_nslp_pin, HIGH);

  digitalWrite(left_dir_pin, LOW);
  digitalWrite(right_dir_pin, LOW);

  pinMode(LED_pin, OUTPUT);
}

void loop()
{
  if (lineSensed == true)
  {
    count++;
    lineSensed = false;
  }
  if (count == enterZone)
  {
    digitalWrite(LED_pin, HIGH);
    enter(30);
  }
  if (count == leaveZone)
  {
    digitalWrite(LED_pin, LOW);
    leave(75);
  }

  float fusion2 = IRSensorInput(lineSensed);
  goPID(fusion2);

}


void goPID(float fusion) //PID Control
{

  //Proportional
  float proportional_correction = KP * fusion;

  //Integral
  /*TimeDifference = time - prevTime;
    float integral_correction = KI * (fusion - prevValue) * TimeDifference;
    prevTime = time;*/

  //Derivative
  //Serial.println(prevValue);
  //Serial.println(fusion);
  float derivative_correction = KD * (fusion - prevValue);

  prevValue = fusion;

  float correction = proportional_correction + derivative_correction;

  if (correction > 0)
  {
    // turn right
    motorControl(velocityMax, velocityMax - correction);
  }
  else
  {
    //turn left
    motorControl(velocityMax + correction, velocityMax);
  }
}


float IRSensorInput(bool &lineSensed)
{
  pinMode(IRSensor0, OUTPUT);
  pinMode(IRSensor1, OUTPUT);
  pinMode(IRSensor2, OUTPUT);
  pinMode(IRSensor3, OUTPUT);
  pinMode(IRSensor4, OUTPUT);
  pinMode(IRSensor5, OUTPUT);
  pinMode(IRSensor6, OUTPUT);
  pinMode(IRSensor7, OUTPUT);

  //Serial.println("d");
  digitalWrite(IRSensor0, HIGH);
  digitalWrite(IRSensor1, HIGH);
  digitalWrite(IRSensor2, HIGH);
  digitalWrite(IRSensor3, HIGH);
  digitalWrite(IRSensor4, HIGH);
  digitalWrite(IRSensor5, HIGH);
  digitalWrite(IRSensor6, HIGH);
  digitalWrite(IRSensor7, HIGH);

  delayMicroseconds(10);
  //Serial.println("b");
  pinMode(IRSensor0, INPUT);
  pinMode(IRSensor1, INPUT);
  pinMode(IRSensor2, INPUT);
  pinMode(IRSensor3, INPUT);
  pinMode(IRSensor4, INPUT);
  pinMode(IRSensor5, INPUT);
  pinMode(IRSensor6, INPUT);
  pinMode(IRSensor7, INPUT);

  //Serial.println("h");

  delayMicroseconds(2000);

  //Serial.println("t");
  int IRVal0 = digitalRead(IRSensor0);
  int IRVal1 = digitalRead(IRSensor1);
  int IRVal2 = digitalRead(IRSensor2);
  int IRVal3 = digitalRead(IRSensor3);
  int IRVal4 = digitalRead(IRSensor4);
  int IRVal5 = digitalRead(IRSensor5);
  int IRVal6 = digitalRead(IRSensor6);
  int IRVal7 = digitalRead(IRSensor7);

  /*Serial.print(IRVal7);
    Serial.print(IRVal6);
    Serial.print(IRVal5);
    Serial.print(IRVal4);
    Serial.print(IRVal3);
    Serial.print(IRVal2);
    Serial.print(IRVal1);
    Serial.print(IRVal0);
    Serial.println();*/

  float fusedValue = (-1.75 * IRVal7) + (-1.25 * IRVal6) + (-0.75 * IRVal5) + (-0.25 * IRVal4) + (0.25 * IRVal3) + (0.75 * IRVal2) + (1.25 * IRVal1) + (1.75 * IRVal0);

  int stopValue = IRVal7 + IRVal5 + IRVal4 + IRVal3 + IRVal2 + IRVal1 + IRVal0;

  if (stopValue >= 4)
    lineSensed = true;
  else
    lineSensed = false;

  return fusedValue;

}

void motorControl(float leftMotor, float rightMotor)
{
  analogWrite(left_pwm_pin, leftMotor);
  analogWrite(right_pwm_pin, rightMotor);
}

/*void threeSixty()
  {
  digitalWrite(left_dir_pin, HIGH);
  digitalWrite(right_dir_pin, LOW);
  unsigned long startTime = millis();
  unsigned long endTime = startTime;
  while ((endTime - startTime) <= 600)
  {
    motorControl(200, 200);
    endTime = millis();
  }
  digitalWrite(right_dir_pin, LOW);
  digitalWrite(left_dir_pin, LOW);
  }*/

  void enter(int idealSpeed)
  {
  float timeAvg = (diffLA + diffRA) / 2;

  float errorSpeed = (distanceMeasured / timeAvg);

  //Serial.println(errorSpeed);
  errorSpeed *= 300;

  if (errorSpeed > idealSpeed)
  {
    velocityMax -= 10;
  }
  }

  void leave(int idealSpeed)
  {
  float timeAvg = (diffLA + diffRA) / 2;

  float errorSpeed = (distanceMeasured / timeAvg);

  //Serial.println(errorSpeed);
  errorSpeed *= 300;

  if (errorSpeed < idealSpeed)
  {
    velocityMax += 10;
  }
  }

/*void generateEncoderSpeed(int idealSpeed)
{
  float timeAvg = (diffLA + diffRA) / 2;

  float errorSpeed = (distanceMeasured / timeAvg);

  float error = abs(idealSpeed) - abs(conversionConstant * errorSpeed);

  float proportional_correction = KP2 * error;

  //Integral
  TimeDifference = time - prevTime;
    float integral_correction = KI * (fusion - prevValue) * TimeDifference;
    prevTime = time;

  //Derivative
  //Serial.println(prevValue);
  //Serial.println(fusion);
  //float derivative_correction = KD * (fusion - prevValue);

  float correction = proportional_correction;

  velocityMax = velocityMax + correction;
  //Serial.println(errorSpeed);

}*/
