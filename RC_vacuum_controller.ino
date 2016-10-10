/*
RC Vacuum Cleaner Controller Code

Pin Assignment
D0 
D1 
D2 Vacuum Motor Control (Green pushbutton) 10k pull up resistor
D3 Right Turn (Right blue pushbutton) 10k pull up resistor
D4 Left Turn (Left blue pushbutton) 10k pull up resistor
D5 Accelerate (Red pushbutton) 10k pull up resistor
D6 Decelerate (Black pushbutton) 10k pull up resistor
D7 
D8 NRF24 CE
D9 
D10 NRF24 CSN
D11 NRF24 MOSI
D12 NRF24 MISO
D13 NRF24 SCK
A0 Motor Speed Control (10k potentiometer voltage divider)
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
const byte VACUUM = 2;
const byte RIGHT = 3;
const byte LEFT = 4;
const byte ACCELERATE = 5;
const byte DECELERATE = 6;
const byte POT = A0;

// Define variables
byte buttons_current = 0b00000000;  // Holds current pushbutton command set
byte buttons_previous = 0b00000000; // Holds preivous pushbutton command set
byte pot_current = 0x00;  // Holds current voltage division of potentiometer
byte pot_previous = 0x00;  // Holds previous voltage division of potentiometer


// Vehicle ID
const byte ID0 = 0x53; // 'S'
const byte ID1 = 0x42; // 'B'
const byte ID2 = 0x34; // '4'
const byte ID3 = 0x32; // '2'

RF24 radio(CE, CSN);  // Create RF24 object

const uint64_t controller = 0xE8E8F0F0E1LL; // Controller address

/*
 * Data storage for command set
 * 
 * command[0 TO 3] = vehicle ID
 * 
 * command[4] = 8-bit instruction set
 *    - 0b00000000 (MSB 7 DOWNTO 0)
 *    - bit[7 DOWNTO 5] = Blank instructions. N/A.
 *    - bit[4] = accelerate
 *    - bit[3] = decelerate
 *    - bit[2] = left turn
 *    - bit[1] = right turn
 *    - bit[0] = vacuum switch
 *    
 * command[5] = motor speed control (0 to 255. 0 is slowest, 255 is fastest)
 */
byte command[6];

void setup() {
  Serial.begin(9600);

  // Initialize vehicle ID in command set
  command[0] = ID0;
  command[1] = ID1;
  command[2] = ID2;
  command[3] = ID3;

  // Initialize 8-bit instruction set
  command[4] = 0b00000000;

  // Initialize motor speed to slowest
  command[5] = 0x00;

  radio.begin();  // Initialize NRF24 radio module
  radio.openWritingPipe(controller);  // Set address of controller
}

void loop() {
  check_buttons();
  check_pot();
}

/*
 * Button Command Set Control
 * 
 * Checks the pushbuttons. If any of the buttons are pressed,
 * send the corresponding commands to the receiver/vehicle.
 * Will send one byte of information containing 5 separate
 * commands.
 */

void check_buttons() {
  // Read button input and store in 8-bit button command set
  buttons_current = 0b00000000;
  buttons_current += debounce(ACCELERATE, (buttons_previous & 0b00010000) == 0b00010000) ? 0b00010000 : 0b00000000;
  buttons_current += debounce(DECELERATE, (buttons_previous & 0b00001000) == 0b00001000) ? 0b00001000 : 0b00000000;
  buttons_current += debounce(LEFT, (buttons_previous & 0b00000100) == 0b00000100) ? 0b00000100 : 0b00000000;
  buttons_current += debounce(RIGHT, (buttons_previous & 0b00000010) == 0b00000010) ? 0b00000010 : 0b00000000;
  buttons_current += debounce(VACUUM, (buttons_previous & 0b00000001) == 0b00000001) ? 0b00000001 : 0b00000000;

  // Only send data to receiver if data has changed
  if (buttons_current != buttons_previous) {
    command[4] = buttons_current;
    send_data();

    // Store the sent values;
    buttons_previous = buttons_current;
  }
}

/* 
 * Motor Speed Control
 * 
 * Checks the potentiometer and sends the voltage output from
 * its voltage divider to the receiver/vehicle.
 * Potentiometer is what controls the motor's speed.
 */
void check_pot() {
  // Read voltage from potentiometer, then map value from range
  // 0 - 1023 to 170 - 255.
  pot_current = map(analogRead(POT), 0, 1023, 0xAA, 0xFF);
  
  // Only send data to receiver if data has chagned.
//  if (pot_current < pot_previous - 5 || pot_current > pot_previous + 5) {
  if (pot_current != pot_previous) {
    command[5] = pot_current;
    send_data();

    // Store the sent values
    pot_previous = pot_current;
  }
}

/* 
 *  Data Send
 *  
 *  Transmits data (command set) to the receiver/vehicle.
 */
void send_data() {
  radio.write(command, sizeof(command));
}

/*
 * Debouncing
 * 
 * Pass it previous button state,
 * and get back current debounced button state
 * 
 * @param button_pin - Pin number of button to debounce
 * @param last_button_state - Respective button's last state
 * @return - Debounced state of button
 */
bool debounce(byte button_pin, bool last_button_state) {
  bool current_state = digitalRead(button_pin);
  if (current_state != last_button_state) {
    delay(5); // wait 5ms (to double check button for assurance)
    current_state = digitalRead(button_pin);
  }
  return current_state;
}


