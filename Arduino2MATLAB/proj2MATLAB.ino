// Arthor Tim Wong (638207)
// This sketch uses ADXL-335 sensor and counts no. of step taken as a person is walking.
// Acceleration data is sent to a serial port and intended to be read using MATLAB code.

// Pin setting
const int groundpin = A0;             // analog input pin 4 -- ground
const int powerpin = A4;              // analog input pin 5 -- voltage
const int xpin = A3;                  // x-axis of the accelerometer
const int ypin = A2;                  // y-axis
const int zpin = A1;                  // z-axis (only on 3-axis models)

// Variables decleration 
float g = 9.81;
unsigned long currentTime = 0;
float prevTime = 0;
float t = 0;

// For counting step
const int stepAcc = -11; //Acceleration in z direction to be counted as a step
const double timeFrame = 0.4; //Time frame to count step
boolean counted = false;
double timer = 0;
int stepCount = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  establishContact();
  pinMode(groundpin, OUTPUT);
  pinMode(powerpin, OUTPUT);
  digitalWrite(groundpin, LOW);
  digitalWrite(powerpin, HIGH);
}

void loop() {
  handleSerial();
  // Sensor values:
}


// A function to establish connection
void establishContact(){
  while (Serial.available() <= 0){
    Serial.println('established');
    delay(1000);
  }
}

// A function to handle serial communication within the loop function
void handleSerial(){
  int inByte;
  if(Serial.available() > 0){
    //get incoming instruction:
    inByte = Serial.read();
    //Handle instruction
    if(inByte == 'R'){// Read sensor values to MATLAB
      
      // Sensor values:
      float aX = -map(analogRead(xpin),281,424,-1000,1000)/1000.0;
      float aY = -map(analogRead(ypin),280,420,-1000,1000)/1000.0;
      float aZ = -map(analogRead(zpin),290,433,-1000,1000)/1000.0;

      // Time stamp
      currentTime = millis();
      t = (currentTime - prevTime)/1000;
      prevTime = currentTime;

      //Acceleration values in ms^-2
      float ax = aX*g;
      float ay = aY*g;
      float az = aZ*g;

      //Step counting
      timer = timer - t;
      if(counted && timer < 0){
        counted = false;
      }
      if(az<= stepAcc && !counted){
        stepCount = stepCount + 1;
        timer = timeFrame;
        counted = true;
      }
      
      //Sending data to MATLAB
      Serial.println(ax);
      Serial.println(prevTime);
      Serial.println(stepCount);
      delay(10);
    }
  }
}

