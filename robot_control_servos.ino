/*  GeekBot Advanced for BrainOS Firmware for GeekDuino / control code

    ===============================================================================
    Copyright (c) 2015, Brain Corporation
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
       * Redistributions of source code must retain the above copyright
         notice, this list of conditions and the following disclaimer.
       * Redistributions in binary form must reproduce the above copyright
         notice, this list of conditions and the following disclaimer in the
         documentation and/or other materials provided with the distribution.
       * Neither the name of Brain Corporation nor the
         names of its contributors may be used to endorse or promote products
         derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL BRAIN CORPORATION BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
    ===============================================================================


    Expects pins connected on the GeekDuino sensor shield:

    Grip Servo: servo pin 9
    Wrist Servo: servo pin 10
    Camera Servo: servo pin 6
    Left Wheel Servo: servo pin 3
    Right Wheel Servo: servo pin 5

    FSR Grip Sensors: Analog Input Pins 0 and 1

    Serial protocol:

    The serial protocol that it uses to communicate with the Arduino is includes
    SET and GET messages.  SET messages are of the form
    +---+----------------------+--------+----+
    | = | command character    | value  | ;  |
    +---+----------------------+--------+----+
    and GET commands are of the (shorter) form
    +---+----------------------+
    | = | command character    |
    +---+----------------------+
    For example, to set the gripper to position 30, send the string "=G30;".
    +---+----+---+---+
    | = | G  |30 | ; |
    +---+----+---+---+
    The possible GET and SET commands are listed in the following table:
    +---------------------+-----------------------------------------+----------+
    | Command Character   | Meaning                                 | Range    |
    +=====================+=========================================+==========+
    |    L                | SET left servo rotation speed           | 0 .. 180 |
    +---------------------+-----------------------------------------+----------+
    |    R                | SET right servo rotation speed          | 0 .. 180 |
    +---------------------+-----------------------------------------+----------+
    |    W                | SET wrist vertical position             | 0 .. 180 |
    +---------------------+-----------------------------------------+----------+
    |    G                | SET gripper open/close position         | 0 .. 180 |
    +---------------------+-----------------------------------------+----------+
    |    C                | SET camera position                     | 0 .. 180 |
    +---------------------+-----------------------------------------+----------+
    |    M                | SET max gripper voltage (10x actual val)| 0 .. 50  |
    +---------------------+-----------------------------------------+----------+
    |    V                | GET gripper voltage                     | 0 .. 5.0 |
    +---------------------+-----------------------------------------+----------+
*/

#include <Servo.h>

#define LEFT_SERVOPIN 3  //pin that the left servo will be attached to
#define RIGHT_SERVOPIN 5  //pin that the right servo will be attached to
#define CAMERA_SERVOPIN 6
#define GRIP_SERVOPIN 9  //pin that the micro servo will be attached to
#define WRIST_SERVOPIN 10  //pin that the large servo will be attached to

Servo leftServo;  // create servo object to control left wheel continuous turn servo
Servo rightServo;  // create servo object to control right wheel continuous turn servo

Servo gripServo;   //create an servo object for the 9g FT-FS90MG micro servo
Servo wristServo;   //create an servo object for the RobotGeek 180 degree serco

Servo camServo;

bool gripForceFeedback = true;
float maxGripVoltage = 4.5; // Maximum allowed grip strength (voltage from gripper FSR).
float cmdUpdateRate = 0.02;
float cmd = 10.0;
float lastVoltage = 0.0;
int left_pos;    // variable to store left wheel continuous turn servo position
int right_pos;    // variable to store right wheel continuous turn servo position
int grip_pos = 10;  // desired (target) grip position
int wrist_pos;
int cam_pos = 90;

void setup()
{
  leftServo.attach(LEFT_SERVOPIN);
  rightServo.attach(RIGHT_SERVOPIN);
  gripServo.attach(GRIP_SERVOPIN);
  wristServo.attach(WRIST_SERVOPIN);
  camServo.attach(CAMERA_SERVOPIN);
  Serial.begin(115200);
  delay(2000);while(!Serial);
  Serial.print("Initialized!\n");
}

void gripPosControl()
{
   //    should try to set grip to grip_pos,
   //    but if force sensor voltage exceeds maxGripVoltage, engage control.

    int sensorValue0 = analogRead(A0);
    int sensorValue1 = analogRead(A1);
    int sensorValue = max(sensorValue0, sensorValue1);
    float voltage = sensorValue * (5.0 / 1023.0);

    lastVoltage = voltage;
    if(voltage == 0.0)
    {
          cmd = grip_pos;
    }
    else
    {
          if (voltage > maxGripVoltage)
          {
            cmd -= cmdUpdateRate;
          }
          else
          {
            if(cmd < grip_pos)
            {
              cmd += cmdUpdateRate;
            }
         }
    }
     gripServo.write(180-cmd);
}

void loop()
{
    gripPosControl();

    if (Serial.available() > 0)
    {
      char command = Serial.read();

      if (command == '=')
      {
        while(!Serial.available()){}
        char command = Serial.read();
        if (command == 'L')
        {
            writeLeftPos();
            leftServo.write(left_pos);
        }
        if (command == 'R')
        {
             writeRightPos();
             rightServo.write(right_pos);
        }
        if (command == 'W')
        {
             writeWristPos();
             wristServo.write(wrist_pos);
        }
        if (command == 'G')
        {
             writeGripPos();
             gripServo.write(180-grip_pos);
             cmd = grip_pos;
        }
        if (command == 'M')
        {
             writeMaxGripV();
        }
        if (command == 'V')
        {
            Serial.print(lastVoltage);
            Serial.print("\n");
        }
        if (command == 'C')
        {
             writeCamPos();
             camServo.write(cam_pos);
        }
      }
    }
}

void writeCamPos() {
       bool receiving = true;
       String buffer = "";
        while (receiving) {
              if (Serial.available() > 0) {
              char newCharacter = Serial.read();
              switch (newCharacter) {
                case ';':
                  receiving=false;
                  break;
                default:
                  buffer = buffer + newCharacter;
              }
            }
          }

          cam_pos = buffer.toInt();
}

void writeWristPos() {
       bool receiving = true;
       String buffer = "";
        while (receiving) {
              if (Serial.available() > 0) {
              char newCharacter = Serial.read();
              switch (newCharacter) {
                case ';':
                  receiving=false;
                  break;
                default:
                  buffer = buffer + newCharacter;
              }
            }
          }

          wrist_pos = buffer.toInt();
}

void writeGripPos() {
       bool receiving = true;
       String buffer = "";
        while (receiving) {
              if (Serial.available() > 0) {
              char newCharacter = Serial.read();
              switch (newCharacter) {
                case ';':
                  receiving=false;
                  break;
                default:
                  buffer = buffer + newCharacter;
              }
            }
          }

          grip_pos = buffer.toInt();
}

void writeMaxGripV() {
       bool receiving = true;
       String buffer = "";
        while (receiving) {
              if (Serial.available() > 0) {
              char newCharacter = Serial.read();
              switch (newCharacter) {
                case ';':
                  receiving=false;
                  break;
                default:
                  buffer = buffer + newCharacter;
              }
            }
          }
          // max grip voltage (int: 10x actual value)
          int maxGripV10x = buffer.toInt();
          maxGripVoltage = ((float) maxGripV10x) * 0.1;
          Serial.print(maxGripVoltage);
          Serial.print("\n");
}

void writeLeftPos() {
       bool receiving = true;
       String buffer = "";
        while (receiving) {
              if (Serial.available() > 0) {
              char newCharacter = Serial.read();
              switch (newCharacter) {
                case ';':
                  receiving=false;
                  break;
                default:
                  buffer = buffer + newCharacter;
              }
            }
          }

          left_pos = buffer.toInt();
}

void writeRightPos() {
       bool receiving = true;
       String buffer = "";
        while (receiving) {
              if (Serial.available() > 0) {
              char newCharacter = Serial.read();
              switch (newCharacter) {
                case ';':
                  receiving=false;
                  break;
                default:
                  buffer = buffer + newCharacter;
              }
            }
          }

          right_pos = buffer.toInt();
}


