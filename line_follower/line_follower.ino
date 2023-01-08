/*RoboTutor V3.0*/
/*IEEE BUET Student Branch*/

//****************************************************************
// MOTOR PARAMETERS
//****************************************************************
#define   inA         9
#define   inB         8
#define   inC         6
#define   inD         7

#define   enA         5
#define   enB         10

int leftBaseSpeed   = 100;
int rightBaseSpeed  = 100;
int maxSpeed        = 120;
//****************************************************************




//****************************************************************
// SENSOR PARAMETERS
//****************************************************************
#define NUM_SENSORS   5

int sensorValues[NUM_SENSORS], lastSensor;
int sensorPins[NUM_SENSORS] = {A4, A3, A2, A1, A0};
//****************************************************************



//****************************************************************
// PID PARAMETERS
//****************************************************************
float kp            = 2;
float kd            = .5;
int prevError;
//****************************************************************

void setup()
{  
  blink();
  //Initialize Motor Pins
  motorInit();
  //Initialize Other Variables and Pins
  otherInit();
}
void loop()
{
  lineFollow();
  
}

void motorInit()
{
  pinMode(inA, OUTPUT);
  pinMode(inB, OUTPUT);
  pinMode(inC, OUTPUT);
  pinMode(inD, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);


  //Setting The Motor to zero speed
  digitalWrite(inA, HIGH);
  digitalWrite(inB, HIGH);
  digitalWrite(inC, HIGH);
  digitalWrite(inD, HIGH);
}

void otherInit()
{
  lastSensor = 0;
  prevError = 0;
  Serial.begin(9600);
}


void lineFollow()
{
  int error, delSpeed;
  float P, D;

  //Read the sensor values and calculate error
  error = readSensor();

  //If no sensors are on line
  if (error == 420)
  {
    //If previously left sensor was on the line, turn left
    if (lastSensor == 1) wheel(-180, 180);
    //Else if the previously right sensor on the line, turn right
    else if (lastSensor == 2) wheel(180, -180);
  }

  else
  {
    //Calculate P and D
    P = kp * error;
    D = kd * (error - prevError);

    //Calculating change in speed
    delSpeed = P + D;

    //Setting the motors
    wheel(leftBaseSpeed + delSpeed, rightBaseSpeed - delSpeed);

    //Saving the current error
    prevError = error;
  }
}


int readSensor()
{
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    //Read the raw sensor values
    sensorValues[i] = !digitalRead(sensorPins[i]);
  }

  int error, sumS, sumWS, linePos;

  //Calculate number of sensors currently on the line
  sumS = sensorValues[0] + sensorValues[1] + sensorValues[2] + sensorValues[3] + sensorValues[4];

  //If no sensors are on the line, return a specified signal
  if (sumS == 0) return 420;

  //Calculate line position
  sumWS = (sensorValues[0] * 80 + sensorValues[1] * 70 + sensorValues[2] * 50 + sensorValues[3] * 30 + sensorValues[4] * 20);
  linePos = (sumWS / sumS);

  //Calculate the error
  error = linePos - 50;

  //Keep track of which on of the two extreme sensors was on the line
  //1 == Leftmost sensor, 2 == rightmost sensor
  if (sensorValues[0] == 1) lastSensor = 1;
  else if (sensorValues[4] == 1) lastSensor = 2;

  //Return the error
  return error;
}


void wheel(int leftSpeed, int rightSpeed)
{
  Serial.print(leftSpeed);
  Serial.print(' ');
  Serial.println(rightSpeed);
  if ( leftSpeed == 0)
  {
    digitalWrite(inC, LOW);
    digitalWrite(inD, LOW);
  }
  if ( leftSpeed > 0)
  {
    digitalWrite(inC, HIGH);
    digitalWrite(inD, LOW);
  }
  else if ( leftSpeed < 0)
  {
    digitalWrite(inC, LOW);
    digitalWrite(inD, HIGH);
  }


  if ( rightSpeed == 0)
  {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
  }
  if ( rightSpeed > 0)
  {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  }
  else if ( rightSpeed < 0)
  {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
  }
  if (abs(leftSpeed) > maxSpeed) leftSpeed = maxSpeed;
  if (abs(rightSpeed) > maxSpeed) rightSpeed = maxSpeed;

  analogWrite(enA, abs(rightSpeed));
  analogWrite(enB, abs(leftSpeed));

}

void blink(){
  pinMode(13, OUTPUT);
  for(int i=0; i<3; i++){  
    digitalWrite(13, HIGH);
    delay(500);
    digitalWrite(13, LOW);
    delay(500); 
  } 
}
