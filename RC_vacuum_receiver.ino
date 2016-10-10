/*
RC Vacuum Receiver code

Pin Assignment
D0 
D1 
D2 
D3 Motor 1 Backward
D4 Fan Motor (Vacuum)
D5 Motor 1 Forward
D6 Motor 2 Forward
D7 Motor Enable
D8 NRF24 CE
D9 Motor 2 Backward
D10 NRF24 CSN
D11 NRF24 MOSI
D12 NRF24 MISO
D13 NRF24 SCK
A0 
A1 
A2 
A3 
A4 
A5 
*/

#include <SPI.h>
#include <RF24.h>

// Pin assignments
const byte CE = 8;
const byte CSN = 10;
const byte M1_FORWARD = 5;
const byte M1_BACKWARD = 3;
const byte M2_FORWARD = 6;
const byte M2_BACKWARD = 9;
const byte ENABLE = 7;
const byte FAN = 4;

// Vehicle ID
const byte ID0 = 0x53; // 'S'
const byte ID1 = 0x42; // 'B'
const byte ID2 = 0x34; // '4'
const byte ID3 = 0x32; // '2'

RF24 radio(CE, CSN);  // Create RF24 object

const uint64_t controller = 0xE8E8F0F0E1LL; // Controller address

// Declare variables
bool vacuum_current = LOW;  // LOW = vacuum button not pressed, HIGH = vacuum button pressed
bool vacuum_previous = LOW; 
bool vacuum_on = false;
byte motor_speed = 0; // Ranges from 170 to 255. 170 = slowest, 255 = fastest.

/* 
 *  Variables that store current/preivous motor action
 *  (accelerate, decelerate, left, right).
 *  Each state can only perform/hold maximum one action at a time.
 *  Ex: 0b00010000 (acceptable, car is accelerating)
 *      0b00011000 (unacceptable, car cannot accelerate and decelerate
 *                  at the same time)
 */  
byte motor_action_current = 0b00000000;
byte motor_action_previous = 0b00000000;

/*
 * Buffer for command set
 * 
 * buf[0 TO 3] = vehicle ID
 * 
 * buf[4] = 8-bit instruction set
 *    - 0b00000000 (MSB 7 DOWNTO 0)
 *    - bit[7 DOWNTO 5] = Blank instructions. N/A.
 *    - bit[4] = accelerate
 *    - bit[3] = decelerate
 *    - bit[2] = left turn
 *    - bit[1] = right turn
 *    - bit[0] = vacuum switch
 *    
 * buf[5] = motor speed control (0 to 255. 0 is slowest, 255 is fastest)
 */
byte buf[6];

void setup() {
  Serial.begin(9600);

//  // Set Motor pin frequencies to 31250
//  setPwmFrequency(M1_FORWARD, 1);

  pinMode(M1_FORWARD, OUTPUT);
  pinMode(M1_BACKWARD, OUTPUT);
  pinMode(M2_FORWARD, OUTPUT);
  pinMode(M2_BACKWARD, OUTPUT);
  pinMode(ENABLE, OUTPUT);
  pinMode(FAN, OUTPUT);

  // Initialize motor enable to low so vehicle doesn't run at startup
  digitalWrite(ENABLE, LOW);

  radio.begin();  // Initialize NRF24 radio module
  radio.openReadingPipe(1, controller);  // Set address of controller
  radio.startListening(); // Start listening for commands from controller
}

void loop() {
  check_data();
  move();
  vacuum_control();
  


  // Check state of vehicle
//  Serial.print("Motor speed = ");
//  Serial.println(motor_speed);
//  Serial.print("Vacuum = ");
//  if (vacuum_on)
//    Serial.println("ON");
//  else
//    Serial.println("OFF");
//
//  Serial.print("Action = ");
//  switch (motor_action_current) {
//  case 0b00010000:
//    Serial.println("ACCELERATE");
//    break;
//  case 0b00001000:
//    Serial.println("DECELERATE");
//    break;
//  case 0b00000100:
//    Serial.println("LEFT");
//    break;
//  case 0b00000010:
//    Serial.println("RIGHT");
//    break;
//  default:
//    Serial.println("NONE");
//    break;
//  }
//  Serial.println();
  
}

/*
 * Data Check
 * 
 * Checks data from the controller.
 * If data is available, store the comamnd set into
 * program's global variables.
 */
void check_data() {
  // If radio is available to be read (command was sent),
  // go ahead and read the command set and store its values.
  if (radio.available()) {
    Serial.println("AVAILABLE!");
    radio.read(buf, sizeof(buf));
    
    // Return the function if vehicle IDs do not match
    if (!check_vehicle_ID()) {
      return;
    }

    // Change vacuum state if vacuum button is recently pressed
    vacuum_current = buf[4] & 0b00000001;
    if (vacuum_current == HIGH && vacuum_previous == LOW) { 
      vacuum_on = !vacuum_on;
    }
    vacuum_previous = vacuum_current;  // Store vacuum state into previous state

    // Store button command values. Maximum of one action can be stored.
    // If no motor buttons are pressed, action is set to none.
    if ((buf[4] >> 1) == 0b00000000) {
//      Serial.println("ACTION PRESSED");
      motor_action_current = 0b00000000;
    }
    // In case of multiple buttons pressed at once, buttons take precedence
    // of (accelerate > decelerate > left > right).
    else if (motor_action_previous == 0b00000000) {
      for (byte action = 0b00010000; action != 0b00000001; action >>= 1) {
        if ((buf[4] & action) == action) {
          motor_action_current = action;
          break;
        }
      }
    }
    motor_action_previous = motor_action_current;
    
    // Store motor speed value
    motor_speed = buf[5];
  }
}

/* 
 * Vehicle ID Verification
 * 
 * Checks if vehicle ID is same from buffer command.
 * 
 * @return True if they are the same, false otherwise.
 */
bool check_vehicle_ID() {
  return buf[0] == ID0 && buf[1] == ID1 && buf[2] == ID2 && buf[3] == ID3;
}

/*
 * Vehicle Movement
 * 
 * Vehicle drives as commanded by the controller. PWM dictates
 * motor speed.
 *  *    - bit[4] = accelerate
 *    - bit[3] = decelerate
 *    - bit[2] = left turn
 *    - bit[1] = right turn
 */
void move() {
  switch (motor_action_current) {
  case 0b00010000:  // Accelerate
    digitalWrite(ENABLE, HIGH);
    analogWrite(M1_BACKWARD, 0);
    analogWrite(M2_BACKWARD, 0);
    analogWrite(M1_FORWARD, motor_speed);
    analogWrite(M2_FORWARD, motor_speed);
    break;
  case 0b00001000:  // Decelerate
    digitalWrite(ENABLE, HIGH);
    analogWrite(M1_FORWARD, 0);
    analogWrite(M2_FORWARD, 0);
    analogWrite(M1_BACKWARD, motor_speed);
    analogWrite(M2_BACKWARD, motor_speed);
    break;
  case 0b00000100:  // Left turn
    digitalWrite(ENABLE, HIGH);
    analogWrite(M1_FORWARD, 0);
    analogWrite(M2_BACKWARD, 0);
    analogWrite(M1_BACKWARD, motor_speed);
    analogWrite(M2_FORWARD, motor_speed);
    break;
  case 0b00000010:  // Right turn
    digitalWrite(ENABLE, HIGH);
    analogWrite(M1_BACKWARD, 0);
    analogWrite(M2_FORWARD, 0);
    analogWrite(M1_FORWARD, motor_speed);
    analogWrite(M2_BACKWARD, motor_speed);
    break;
  default:  // No action/movement
    digitalWrite(ENABLE, LOW);
    analogWrite(M1_BACKWARD, 0);
    analogWrite(M2_BACKWARD, 0);
    analogWrite(M1_FORWARD, 0);
    analogWrite(M2_FORWARD, 0);
  }
}

/*
 * Vacuum Control
 * 
 * Controls the state of the motor controlling the vacuum fan.
 * Writes logic HIGH/LOW to base of transistor that connects
 * to the motor of the fan. HIGH = motor turned on, LOW = motor turned off.
 */
void vacuum_control() {
  digitalWrite(FAN, vacuum_on);
}
