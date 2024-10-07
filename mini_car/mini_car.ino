// #include <Arduino_FreeRTOS.h>

// TaskHandle_t Task1Handle;
// TaskHandle_t Task2Handle;

const int encoderPinA1 = 18; // Interrupt pin 18
const int encoderPinA2 =19;
const int encoderPinB1 = 20; // Interrupt pin 19
const int encoderPinB2 = 3;
const int MotorL1=8;
const int MotorL2=9;
const int MotorR1=10;
const int MotorR2=11;

// Variables to store the pulse counts
volatile long pulseCountA = 0;
volatile long pulseCountB = 0;

// ISR for Encoder A
void encoderISR_A1() {
  if(digitalRead(encoderPinA2)>0){pulseCountA++;}
  else if(digitalRead(encoderPinA2)==0){pulseCountA--;}
  

}

// ISR for Encoder B
void encoderISR_B1() {
  if(digitalRead(encoderPinB2)>0){pulseCountB--;}
  else if(digitalRead(encoderPinB2)==0){pulseCountB++;}
}


void setup() {
  Serial.begin(115200);

  // pinMode(LED_BUILTIN, OUTPUT);

  pinMode(encoderPinA1, INPUT);
  pinMode(encoderPinA2, INPUT);
  
  pinMode(encoderPinB1, INPUT);
  pinMode(encoderPinB2, INPUT);

  // Attach interrupts to the encoder pins
  attachInterrupt(digitalPinToInterrupt(encoderPinA1), encoderISR_A1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinB1), encoderISR_B1, RISING);

  pinMode(MotorL1, OUTPUT);
  pinMode(MotorL2, OUTPUT);
  pinMode(MotorR1, OUTPUT);
  pinMode(MotorR2, OUTPUT);

}



void serialread(){
  // const TickType_t xFrequency = pdMS_TO_TICKS(100); // 100 ms period
  // TickType_t xLastWakeTime = xTaskGetTickCount();
  analogWrite(MotorL1, pwmL);
  analogWrite(MotorL2, 0);
  while (Serial.available() > 0) {
    // char receivedChar = Serial.read();
    String input = Serial.readStringUntil('\r');  // Read until \r character
    if (input == "e") {
      // Send encoder values back in the format: "value1 value2\r\n"
      Serial.print(pulseCountA);
      Serial.print(" ");
      Serial.print(pulseCountB);
      Serial.println("\r");  // End the message with \r\n
    }
    if (input.startsWith("m"))
    {
      input.trim();
      input.remove(0, 2);
      // Find the position of the space between val1 and val2
      int spaceIndex = input.indexOf(' ');
      // Extract val1 and val2
      String val1String = input.substring(0, spaceIndex);
      String val2String = input.substring(spaceIndex + 1);
      // Convert to integers
      int motor_l_speed = val1String.toInt();
      int motor_r_speed = val2String.toInt();

      float maxRPM = 150; // Replace with your motor's maximum RPM
      float maxRadPerSec = maxRPM * 2 * PI / 60; // Convert RPM to rad/s
      
      int pwmL = map(motor_l_speed, -maxRadPerSec, maxRadPerSec, -255, 255);
      int pwmR = map(motor_r_speed, -maxRadPerSec, maxRadPerSec, -255, 255);

      if (pwmL >= 0) {
        analogWrite(MotorL1, pwmL);
        analogWrite(MotorL2, 0);
      } else {
        analogWrite(MotorL1, 0);
        analogWrite(MotorL2, -pwmL);
      }
      // Right motor control
      if (pwmR >= 0) {
        analogWrite(MotorR1, pwmR);
        analogWrite(MotorR2, 0);
      } else {
        analogWrite(MotorR1, 0);
        analogWrite(MotorR2, -pwmR);
      }
      Serial.println("Motor speed is updated\r");
    }
    if (input.startsWith("u"))
    {
      input.trim();
      input.remove(0, 2);
      // input.remove(input.length()-2,input.length());

      int firstColonIndex = input.indexOf(':');
      int secondColonIndex = input.indexOf(':', firstColonIndex + 1);
      int thirdColonIndex = input.indexOf(':', secondColonIndex + 1);

      // Extract kp, kd, ki, and ko
      String kpString = input.substring(0, firstColonIndex);
      String kdString = input.substring(firstColonIndex + 1, secondColonIndex);
      String kiString = input.substring(secondColonIndex + 1, thirdColonIndex);
      String koString = input.substring(thirdColonIndex + 1);

      // Convert to integers
      int kp = kpString.toInt();
      int kd = kdString.toInt();
      int ki = kiString.toInt();
      int ko = koString.toInt();


    }
  }
}



void loop(){
  serialread();

}
