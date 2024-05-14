#include <SPI.h>

// Define CS pin
#define CS_X_PIN   A2

// Define STEP and DIR pins
#define STEP_X_PIN   5
#define DIR_X_PIN    A5

// Define DIAG pins
#define DIAG0_PIN    2  // Interrupt pin 0
#define DIAG1_PIN    3  // Interrupt pin 1

// Define register addresses for TMC2240
#define GCONF         0x00
#define IHOLD_IRUN    0x10
#define TPOWERDOWN    0x11
#define TPWMTHRS      0x13
#define TCOOLTHRS     0x14
#define THIGH         0x15
#define XDIRECT       0x2D

#define CHOPCONF      0x6C
#define COOLCONF      0x6D
#define DRV_STATUS    0x6F
#define PWMCONF       0x70
#define SGTHRS        0x74
#define SG4_RESULT 0x75



// SPI settings
SPISettings spiSettings(16000000, MSBFIRST, SPI_MODE3);

volatile bool stallDetected0 = false;
volatile bool stallDetected1 = false;

void setup() {
  SPI.begin(); // Begin SPI communication
  Serial.begin(9600); // Start serial communication for debugging

  pinMode(CS_X_PIN, OUTPUT);
  digitalWrite(CS_X_PIN, HIGH);

  // Set pin modes for STEP, DIR, and DIAG
  pinMode(STEP_X_PIN, OUTPUT);
  pinMode(DIR_X_PIN, OUTPUT);
  pinMode(DIAG0_PIN, INPUT);
  pinMode(DIAG1_PIN, INPUT);

  // Attach interrupts to DIAG pins
  attachInterrupt(digitalPinToInterrupt(DIAG0_PIN), handleStall0, FALLING);
  attachInterrupt(digitalPinToInterrupt(DIAG1_PIN), handleStall1, FALLING);

  // Initialize the driver
  driverSetup(CS_X_PIN);
}

void loop() {
  // Example of moving the motor
  Serial.println("X");
  moveMotor(CS_X_PIN, STEP_X_PIN, DIR_X_PIN, 30000, true);
  delay(1000); // Wait for 1 second
  moveMotor(CS_X_PIN, STEP_X_PIN, DIR_X_PIN, 30000, false);
  delay(1000); // Wait for 1 second

  
}

void handleStall0() {
  stallDetected0 = true;
}

void handleStall1() {
  stallDetected1 = true;
}


void driverSetup(int csPin) {

  //   -----GCONF-----
  uint32_t GCONF_data = 0;
  GCONF_data |= (1 << 7);  // Enable diag0_stall
  GCONF_data |= (1 << 2);  // Sets en_pwm_mode which is neededd for stealthchop2
  // Write the GCONF register
  writeTMC2240(csPin, GCONF, GCONF_data);

  //   -----CHOPCONF-----
  uint32_t CHOPCONF_data = 0;
  

 
  CHOPCONF_data |= (0011 << 0);//toff=3
  CHOPCONF_data |= (10 << 15);//tbl=2
  CHOPCONF_data |= (100 << 4);//hstart=4

  CHOPCONF_data = 0x10410150 | (0x1 << 24) | 0x05;  //copied from initial test
  // Write the CHOPCONF register
  writeTMC2240(csPin, CHOPCONF, CHOPCONF_data);

  
  //   -----PWMCONF-----
  uint32_t PWMCONF_data = 0;
  PWMCONF_data |= (1 << 18);  //set pwm autoscale
  PWMCONF_data |= (1 << 19);  //set pwm autograd
  PWMCONF_data |= (1 << 22);  //set pwm measu sd enable
  //writeTMC2240(csPin, PWMCONF, PWMCONF_data);


  //   -----IHOLD_IRUN-----
  uint32_t IHOLD_IRUN_data = 0;
  uint8_t CurrentRun = 16;
  uint8_t CurrentHold = 16;
  IHOLD_IRUN_data = 0x00060000 | ((uint32_t)(CurrentRun & 0x1F) << 8) | ((uint32_t)CurrentHold & 0x1F);  //copied from initial test
  // Write the IHOLD_IRUN register
  writeTMC2240(csPin, IHOLD_IRUN, IHOLD_IRUN_data);

  //   -----SG4_THRS-----
  uint32_t SGTHRS_data = 0xFF;  // SGTHRS: StallGuard threshold (16)
  writeTMC2240(csPin, SGTHRS, SGTHRS_data);



}

void moveMotor(int csPin, int stepPin, int dirPin, int steps, bool dir) {
  digitalWrite(dirPin, dir); // Set direction
  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(100);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(100);
    if (stallDetected1){
      Serial.println("STALL 1");
      stallDetected1 = 0;
      }
    if (stallDetected0){
      Serial.println("STALL 0");
      stallDetected0 = 0;
    }
    if ((i % 500)==0){
      Serial.println(readTMC2240(csPin,SG4_RESULT));
     
    }
  }
}

// Function to write data to a TMC2240 register
void writeTMC2240(int csPin, byte address, uint32_t data) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(csPin, LOW);
  SPI.transfer(address | 0x80);  // Set the write bit (bit 7)
  SPI.transfer((data >> 24) & 0xFF);
  SPI.transfer((data >> 16) & 0xFF);
  SPI.transfer((data >> 8) & 0xFF);
  SPI.transfer(data & 0xFF);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
}

// Function to read data from a TMC2240 register
uint32_t readTMC2240(int csPin, byte address) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(csPin, LOW);
  SPI.transfer(address & 0x7F);  // Clear the write bit (bit 7)
  uint32_t result = SPI.transfer(0x00) << 24;
  result |= SPI.transfer(0x00) << 16;
  result |= SPI.transfer(0x00) << 8;
  result |= SPI.transfer(0x00);
  digitalWrite(csPin, HIGH);
  SPI.endTransaction();
  return result;
}
