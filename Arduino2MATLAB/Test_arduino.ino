
#include <Servo.h>
#include <math.h>

int sensorValue;
int ledPin = 7;//PWM pin, where to write 
int base_servo_pin = 9;
int base_servo_value = 0;

Servo base_servo;

int com1;
int com2;
int com3;
int com5;
int comClamp;



void setup() {
  // put your setup code here, to run once:
  pinMode(ledPin, OUTPUT);
  base_servo.attach(base_servo_pin);

  Serial.begin(9600);
  establishContact();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  
  sensorValue = analogRead(A0);
  
  transmitData(sensorValue,sensorValue,sensorValue);
  
  readCommand(&com1,&com2,&com3,&com5,&comClamp);
  
  /*
  Test Servo
  */
  
  //base_servo.write(com1);
  //delay(500);
  //base_servo.write(175);
  //delay(500);
  //base_servo.write(175);
  //delay(500);
  //base_servo.write(90);
  //delay(500);
   

  //transmitData(com1,com5,comClamp);

  int ledBrightness = map(sensorValue,0,1023,0,255);
  analogWrite(ledPin, ledBrightness);//+83 //send PWM with dutycycle following ledbrightness
  //Serial.println(sensorValue);
}

void establishContact(){
  while (Serial.available() <= 0){
    Serial.println("established");//send initial string
    delay(300);
  }
}



void transmitData(int pot1, int pot2, int pot3){

  //String stringOne = "{";
  String stringTwo = ",";
  //String stringThree = "}";
  //String stringTotal = stringOne + pot1 + stringTwo + pot2 + stringTwo + pot3 + stringThree;
  String stringTotal = pot1 + stringTwo + pot2 + stringTwo + pot3;
  Serial.println(stringTotal); 
  
}

/*
 
 */
void readCommand(int *q1, int *q2, int *q3, int *q5, int *clamp){

    // if there's any serial available, read it:
  while (Serial.available() > 0) {

    // look for the next valid integer in the incoming serial stream:
    *q1 = Serial.parseInt();
    // do it again:
    *q2 = Serial.parseInt();
    // do it again:
    *q3 = Serial.parseInt();
    *q5 = Serial.parseInt();
    *clamp = Serial.parseInt();

    // look for the newline. That's the end of your sentence:
    //if (Serial.read() == '\n') {
      
      // constrain the values to 0 - 255 and invert
      // if you're using a common-cathode LED, just use "constrain(color, 0, 255);"
      /*red = 255 - constrain(red, 0, 255);
      green = 255 - constrain(green, 0, 255);
      blue = 255 - constrain(blue, 0, 255);*/

      // print the three numbers in one string as hexadecimal:
      /*Serial.print(*q1);
      Serial.print(",");
      Serial.print(*q2);
      Serial.print(",");
      Serial.println(*q3);*/
      
    //}
  }
  
}

String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length()-1;

  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
        found++;
        strIndex[0] = strIndex[1]+1;
        strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}


 /*
 DRAFT PART TO REUSE
 */
 
/*
void handleSerial(){
  int inByte;
  if(Serial.available() > 0){
    //get incoming instruction :
    inByte = Serial.read();
    //handle instruction
    if(inByte == GRIP1_STATUS){
      //Send gripper 1 status
      Serial.println(gripStatus[0]);
    }
  }
}
*/

/*
void readCommand(int *q1, int *q2, int *q3, int *q5, int *clamp){

  String stringTwo = ",";
  double a1, a2, a3, a4, a5;
  a1 = 27.5;
  a2 = 16.8;
  a3 = 3.3;
  a4 = 4.9;
  a5 = 5.1;
  String myStringTest = a1 + stringTwo + a2 + stringTwo + a3 + stringTwo + a4 + stringTwo + a5;
  

  String inString = "";
  
  int inByte = 0;
  int inByte1 = 0;
  String stringByte;
  double command[4];
  if(Serial.available() > 0){
    //inByte  = Serial.read(); // = -1 quand pas d'infos
    stringByte  = Serial.readString();//Serial.read();//
  }*/

  //

 /*   // Read serial input:
  if (Serial.available() > 0) {//while
    int inChar = Serial.read();
    if (isDigit(inChar)) {
      
      // convert the incoming byte to a char and add it to the string:
      inString += (char)inChar;
      
    }
    // if you get a newline, print the string, then the string's value:
    if (inChar == '\n') {
      
      Serial.print("Value:");
      Serial.println(inString.toInt());
      Serial.print("String: ");
      Serial.println(inString);

      //do action on inString
      String part01 = getValue(inString,',',0);
      String part02 = getValue(inString,',',1);

      *q1 = part01.toFloat();//ATTENTION changer en int, pcq les float sont arrondis a cause des points
      *q2 = part02.toFloat();
      
      // clear the string for new input:
      inString = "";
      
    }
  }*/
  
  //

  /*Serial.println(stringByte);

    String part01 = getValue(stringByte,',',0);
    String part02 = getValue(stringByte,',',1);
    String part03 = getValue(stringByte,',',2);
    String part04 = getValue(stringByte,',',3);
    String part05 = getValue(stringByte,',',4);*/
    
    /*Serial.println(part01);
    Serial.println(part02);
    Serial.println(part03);*/
    
  /**q1 = part01.toFloat();//ATTENTION changer en int, pcq les float sont arrondis a cause des points
  *q2 = part02.toFloat();
  *q3 = part03.toFloat();
  *q5 = part04.toFloat();
  *clamp = part05.toFloat();
 
}*/

 
