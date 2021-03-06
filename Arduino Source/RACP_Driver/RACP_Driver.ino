#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// called this way, it uses the default address 0x40
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x41);

#define SERVOMIN_HT  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX_HT  650 // this is the 'maximum' pulse length count (out of 4096)
#define SERVOMIN  150 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  565 // this is the 'maximum' pulse length count (out of 4096)

//Servo zRot, shoulders, elbow, wrist, clawRot, claw;

int numOfJoints = 0;

int oldClawPos = 90;
int clawHoldPos = 90;

SoftwareSerial mySerial(5, 3); //RX, TX

char initChars[1];

//int loopno = 0;

byte index = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(4800);

  pwm.begin();

  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  delay(10);

  delay(10);

 mySerial.begin(9600);
    mySerial.println("Started");
    //unsigned long timer = 0;

  initChars[1] = NULL;

  pinMode(13, OUTPUT);

  while(!Serial.available()) {
    
  }

  if (Serial.available() > 0) {

    //loopno++;

    Serial.readBytes(initChars, 1);


    /*for(int i = 0; i< sizeof(chars); i++) {

      mySerial.print("Character ");
      mySerial.print(i);
      mySerial.print(": ");
      mySerial.println(chars[i]);

      }*/

    String str(initChars);

    numOfJoints = initChars;

  }

  for(int i=0; i<numOfJoints; i++) {

      //initialize all of the servos depending on the number of joints in the arm
      pwm.setPWM(i, 0, (uint16_t) map(90, 0, 180, SERVOMIN, SERVOMAX));
    
  }

}

//calculate num of bytes to receive.
const int recvLen = ((numOfJoints*3) + numOfJoints - 1);

char *strings[63];

char chars[63];

void loop() {
  // put your main code here, to run repeatedly:

  //unsigned long timer = millis();

  if (Serial.available() > (recvLen-1)) {

    //loopno++;

    Serial.readBytes(chars, recvLen);


    /*for(int i = 0; i< sizeof(chars); i++) {

      mySerial.print("Character ");
      mySerial.print(i);
      mySerial.print(": ");
      mySerial.println(chars[i]);

      }*/

    String str(chars);
    /*mySerial.print("In string form: ");
      mySerial.println(str);*/

    char* ptr = NULL;

    index = 0;

    ptr = strtok(chars, ":");

    while (ptr != NULL) {

      /* mySerial.print("Pointer: ");
        mySerial.println(ptr);*/
      strings[index] = ptr;
      index++;
      ptr = strtok(NULL, ":");

    }

    /*mySerial.print("shoulders: "); mySerial.println(map(atoi(strings[1]), 0, 180, SERVOMIN_HT, SERVOMAX_HT));
      mySerial.print("Elbow: "); mySerial.println(map(atoi(strings[2]), 0, 180, SERVOMIN_HT, SERVOMAX_HT));
      mySerial.print("Loop no: "); mySerial.println(loopno);
      mySerial.print("Before writing, this loop took "); mySerial.print(millis()-timer); mySerial.println("ms");*/

    //**Addresses** zRot: 0, Shoulders: 1, Elbow: 2, Wrist: 3, clawRot: 4, claw: 5

    for(int i = 0; i<numOfJoints-1; i++) {

      
    pwm.setPWM(i, 0, (uint16_t) map(atoi(strings[0]), 0, 180, SERVOMIN, SERVOMAX));
      
    }
            digitalWrite(13, LOW);
            //pwm.setPWM(5, 0, (uint16_t) map(atoi(strings[5]), 0, 180, SERVOMIN_HT, SERVOMAX));
mySerial.print("Current: "); mySerial.print(getCurrent()); mySerial.println("mA");

    //mySerial.print("After writing, this loop took "); mySerial.print(millis()-timer); mySerial.println("ms");

    //Pointer for easier reference
    int clawPos = atoi(strings[numOfJoints-1]);

    switch (clawPos) {

      case 90:
      
        if (clawPos != oldClawPos) {

          //Close unless something stops the claw
          for (int i = 0; (i < 90); (i += 5)) {

            //Set servo position
            pwm.setPWM((numOfJoints-1), 0, (uint16_t) map(i, 0, 180, SERVOMIN_HT, SERVOMAX));

            //position to keep if the claw is holding something
            clawHoldPos = (i-10);

            if(getCurrent() > 0.6) {

              pwm.setPWM((numOfJoints-1), 0, (uint16_t) map((i-10), 0, 180, SERVOMIN_HT, SERVOMAX));
              digitalWrite(13, HIGH);
              break;
            }


          }
          
        } else {

          //Set to the previous holding position
          pwm.setPWM((numOfJoints-1), 0, (uint16_t) map(clawHoldPos, 0, 180, SERVOMIN_HT, SERVOMAX));

        }

        break;

        case 0:
        
        pwm.setPWM((numOfJoints-1), 0, (uint16_t) map(clawPos, 0, 180, SERVOMIN_HT, SERVOMAX));

        break;

        default:

        break;

    }

    //update oldClawPos
    oldClawPos = clawPos;

    //mySerial.print("After writing claw position, this loop took "); mySerial.print(millis()-timer); mySerial.println("ms");

  }

  flush();

}

void flush() {

  for (int i = 0; i < recvLen; i++) {

    chars[i] = NULL;
    strings[i] = NULL;

  }

}

//deprecated. remove from functions
double getCurrent() {


  return 0;

}

