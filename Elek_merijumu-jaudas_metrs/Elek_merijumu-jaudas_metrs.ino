#include <Arduino.h>

// Main info
// 2 Modes: 1) DC Power ; 2) AC Power

// Mode info:
// 1) DC P --- Terminals(4): Supply in/out , Load in/out ; ONLY IN SERIES ; SPLY- & LOAD- connected , Voltage(SPLY+/SPLY- --> VD1 --> MCU) , Amperage(SPLY+/LOAD+ --> SHNT --> VD2 --> MCU)
// 2) AC P --- Terminals(4): Supply in/out , Load in/out ; ONLY IN SERIES ; SPLY- & LOAD- connected , Voltage(SPLY+/SPLY- --> FWB1 --> VD1 --> MCU) , Amperage(SHNT --> FWB2 --> VD2 -->  MCU)

// Devices:
// ESP32 (MCU)               ---  PINS: 
// Indicator LED red (LEDR)  ---  For over voltage warning
// Voltage devider (VD1)     ---  For V1
// Voltage devider (VD2)     ---  For V2 (I)
// Shunt resistor (SHNT)     ---  For I
// Full wave bridge 1 (FWB1) ---  For AC V1
// Full wave bridge 2 (FWB2) ---  For AC V2 (I)

// Measured power supply (SPLY)
// Measured load (LOAD)

// Button debuging, leave undefined for no extra info
#define BUTTON_DEBUGING_SERIAL_INFO 

// Leave undifined for no extra timers
#define MORE_TIME_MEASUREMENTS 1

// Analog input buffer size
#define BUFFER_SIZE 200

// Shunt resistors resistance
#define SHUNT_RESISTANCE 0.80436242

// Defins the output to input ratio of voltage dividors (1 / x)
#define VD1_RATIO 10
#define VD2_RATIO 10

// Defines least significant bits
#define LSB_V1 3.3F/4095.0f
#define LSB_V2 3.3F/4095.0f
#define LSB_I LSB_V2
#define LSB_P LSB_V1*LSB_I

// Pins
#define PIN_V1 33
#define PIN_V2 32
#define PIN_LED 16
#define PIN_BUTTON 17

// Stores if button is pressed down
int buttonState;
// Denots the current selected mode
uint16_t selectedMode = 1;

// Buffers for analog measurements (voltage and amperage)
uint16_t bufferV1[BUFFER_SIZE];
uint16_t bufferV2[BUFFER_SIZE];

// Processing variables
uint64_t sumV1 = 0;
uint64_t sumV2 = 0;

// Unique to AC
int64_t rmsSumV1 = 0;
int64_t rmsSumV2 = 0;
int64_t rmsSumI = 0;
int64_t realPowerSum = 0;

// Output variables
float spikeMaxV1 = 0;
float spikeMaxV2 = 0;

float dipMinV1 = 4095;
float dipMinV2 = 4095;

float avgV1 = 0;
float avgV2 = 0;

float avgI = 0;

// Unique to DC
float avgP = 0;

// Unique to AC
float rmsV1 = 0;
float rmsV2 = 0;
float rmsI = 0;
float realPower = 0;
float apparentPower = 0;
float reactivePower = 0;
float powerFactor = 0;

// Only executed on power on
void setup() {
  // Starts serial monitor, with baud rate of 115200
  Serial.begin(115200);
  Serial.println("// Start up begin //");

  // Initiates the pins
  pinMode(PIN_V1, INPUT);
  pinMode(PIN_V2, INPUT);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  // Turns the LED off
  digitalWrite(PIN_LED, LOW);
  
  // Measures the length of the setup function in milliseconds
  unsigned long millisOut = millis();
  Serial.print("// Start up done in ");
  Serial.print(millisOut);
  Serial.println("ms //");
}

void loop() {
  
  Serial.print("\n// New loop in Mode: ");
  Serial.print(selectedMode);
  Serial.println(" //");
  unsigned long loopMillis = millis();

  // Only adds extra timer variables, if extra timers ar on
  #ifdef MORE_TIME_MEASUREMENTS
    unsigned long readerMicros;
    unsigned long processingMicros;
  #endif
  
  // Resets variables, so last cycle dosn't effect this cycle
  varResetter();  

  // Extra timer for reader starts
  #ifdef MORE_TIME_MEASUREMENTS
    readerMicros = micros();
  #endif

  // Reads and writes analog inputs, handels button and LED functions 
  inputReader();

  // Extra timer for inputReader ends and dataProcessor starts
  #ifdef MORE_TIME_MEASUREMENTS
    readerMicros = micros() - readerMicros;
    Serial.print("// Reader done in ");
    Serial.print(readerMicros);
    Serial.println(" us //");

    processingMicros = micros();
  #endif

  // Processes analog data
  dataProcessor();

  // Extra timer for dataProcessor ends
  #ifdef MORE_TIME_MEASUREMENTS
    processingMicros = micros() - processingMicros;
    Serial.print("// Processing done in ");
    Serial.print(processingMicros);
    Serial.println(" us //");
  #endif

  // Prints outputs to serial monitor
  outputPrinter();

  Serial.print("// Loop done in ");
  loopMillis = millis() - loopMillis;
  Serial.print(loopMillis);
  Serial.println("ms //\n");
}

// Resets variables, so last cycle dosn't effect this cycle
void varResetter(){
  
  // Processing variables
  sumV1 = 0;
  sumV2 = 0;

  // Unique to AC
  rmsSumV1 = 0;
  rmsSumV2 = 0;
  rmsSumI = 0;
  realPowerSum = 0;

  // Output variables
  spikeMaxV1 = 0;
  spikeMaxV2 = 0;

  dipMinV1 = 4095;
  dipMinV2 = 4095;

  avgV1 = 0;
  avgV2 = 0;

  avgI = 0;

  // Unique to DC
  avgP = 0;

  // Unique to AC
  rmsV1 = 0;
  rmsV2 = 0;
  rmsI = 0;
  realPower = 0;
  apparentPower = 0;
  reactivePower = 0;
  powerFactor = 0;

}

// Reads and writes analog inputs, handels button and LED functions
void inputReader(){
  // 4 = button was pressed down / 2 = button was up
  int buttonLatch;
  bool ledStillOn;

  // Cycle every buffer entry
  for(int i = 0; i < BUFFER_SIZE; i++){
    // Reads button state 
    buttonState = digitalRead(PIN_BUTTON);
    ledStillOn = false;

    // Button checker
    // Only execute every 10th cycle
    if (i % 10 == 0){
      // Switch statement uses a hash, that is comprised of the current button state and the previous for cycles button state
      switch (buttonState + buttonLatch){
        default:
          buttonLatch = 2;
          Serial.println("/// BUTTON HASH MISMATCH ERROR ///");
        break;

        case 3:
          // Not pressed / wasn't pressed
          #ifdef BUTTON_DEBUGING_SERIAL_INFO
            // Serial.println("No button activity");
          #endif
        break;

        case 2:
          // Is pressed / wasn't pressed
          Serial.println("BUTTON CLICK!!!");

          buttonLatch = 4;

          ++selectedMode;
          if(selectedMode >= 3){
            selectedMode = 1;
          }

        break;

        case 5:
          // Isn't pressed / was pressed
          buttonLatch = 2;
          #ifdef BUTTON_DEBUGING_SERIAL_INFO
            // Serial.println("BUTTON RESET");
          #endif
        break;

        case 4:
          // Is pressed / was pressed
          #ifdef BUTTON_DEBUGING_SERIAL_INFO
            // Serial.println("Button is held down");
          #endif
        break;
      }

    }

    // Read and store an analog value for each buffer entry
    bufferV1[i] = analogRead(PIN_V1);
    bufferV2[i] = analogRead(PIN_V2);
    // Every buffer entry together
    sumV1 += bufferV1[i];
    sumV2 += bufferV2[i];
    
    // Check every buffer entry and store the highest and lowest ones
    if((float)bufferV1[i] > spikeMaxV1){
      spikeMaxV1 = (float)bufferV1[i];
    }
    else if ((float)bufferV1[i] < dipMinV1){
      dipMinV1 = (float)bufferV1[i];
    }

    if((float)bufferV2[i] > spikeMaxV2){
      spikeMaxV2 = (float)bufferV2[i];
    }
    else if ((float)bufferV2[i] < dipMinV2){
      dipMinV2 = (float)bufferV2[i];
    }

    // Over voltage warning checker
    if(bufferV1[i] > 3700){
      digitalWrite(PIN_LED, HIGH);
      ledStillOn = true;
      Serial.println("/// V1 IS OVER 3V ///");
    }
    if(bufferV2[i] > 3700){
      digitalWrite(PIN_LED, HIGH);
      ledStillOn = true;
      Serial.println("/// V2 IS OVER 3V ///");
    }
    if(ledStillOn == false){
      digitalWrite(PIN_LED, LOW);
    }

    delayMicroseconds(200);
  }

}

// Processes analog data
void dataProcessor(){
  // Calculate average by deviding the sum of all buffer entries by the buffer entry count
  avgV1 = sumV1/(float)BUFFER_SIZE;
  avgV2 = sumV2/(float)BUFFER_SIZE;

  // Depending on selected mode, calculate different things
  switch(selectedMode){
    case 1:
      // If DC mode
      // Calculate average current and power
      avgI = (avgV1 - avgV2) / SHUNT_RESISTANCE;
      avgP = avgV1 * avgI;
    break;

    case 2:
      // If AC mode
      // Cycle through all buffer entries
      for(int i = 0; i < BUFFER_SIZE; i++){
        // Sum all buffer entries after squaring them
        rmsSumV1 += (int64_t)(bufferV1[i]*bufferV1[i]);
        rmsSumV2 += (int64_t)(bufferV2[i]*bufferV2[i]);

        // Calculate the sum of all RMS currents and powers
        rmsSumI += (int64_t)( (bufferV1[i] - bufferV2[i]) / SHUNT_RESISTANCE );
        realPowerSum += (int64_t)(bufferV1[i] * ((bufferV1[i] - bufferV2[i]) / SHUNT_RESISTANCE ) );
      } 

      // Calculate the AC specific variables
      rmsV1 = sqrt(rmsSumV1/(float)BUFFER_SIZE);
      rmsV2 = sqrt(rmsSumV2/(float)BUFFER_SIZE);
      rmsI = sqrt(rmsSumI/(float)BUFFER_SIZE);
      realPower = realPowerSum/(float)BUFFER_SIZE;
      apparentPower = rmsV1*rmsI;
      reactivePower = sqrt(apparentPower*apparentPower-realPower*realPower);
      powerFactor = realPower/apparentPower;
    break;

    default:
      // If mode is out of range (1 or 2), return error
      Serial.println("/// SELECTED MODE ERROR ///");
    break;
  }
}

// Prints outputs to serial monitor
void outputPrinter(){
  // General info for voltage 1
  Serial.print("Max spike of V1: ");
  Serial.print(spikeMaxV1 * LSB_V1 * VD1_RATIO);
  Serial.println(" V");

  Serial.print("Min dip of V1: ");
  Serial.print(dipMinV1 * LSB_V1 * VD1_RATIO);
  Serial.println(" V");

  Serial.print("Average V1: ");
  Serial.print(avgV1 * LSB_V1 * VD1_RATIO);
  Serial.println(" V");

  // General info for voltage 2
  Serial.print("Max spike of V2: ");
  Serial.print(spikeMaxV2 * LSB_V2 * VD2_RATIO);
  Serial.println(" V");

  Serial.print("Min dip of V2: ");
  Serial.print(dipMinV2 * LSB_V2 * VD2_RATIO);
  Serial.println(" V");

  Serial.print("Average V2: ");
  Serial.print(avgV2 * LSB_V2 * VD2_RATIO);
  Serial.println(" V");

  switch(selectedMode){
    case 1:
      // DC info
      Serial.print("Average I: ");
      Serial.print(avgI * LSB_I * VD2_RATIO);
      Serial.println(" A");

      Serial.print("Average P: ");
      Serial.print(avgP * LSB_P * VD1_RATIO * VD2_RATIO);
      Serial.println(" W");
    break;

    case 2:
      // AC info
      Serial.print("RMS of V1: ");
      Serial.print(rmsV1 * LSB_V1 * VD1_RATIO);
      Serial.println(" V");

      Serial.print("RMS of V2: ");
      Serial.print(rmsV2 * LSB_V2 * VD2_RATIO);
      Serial.println(" V");

      Serial.print("RMS of I: ");
      Serial.print(rmsI * LSB_I * VD2_RATIO);
      Serial.println(" A");

      Serial.print("Real power: ");
      Serial.print(realPower * LSB_P * VD1_RATIO * VD2_RATIO);
      Serial.println(" W");

      Serial.print("Apparrent power: ");
      Serial.print(apparentPower * LSB_P * VD1_RATIO * VD2_RATIO);
      Serial.println(" VA");

      Serial.print("Reactive power: ");
      Serial.print(reactivePower * LSB_P * VD1_RATIO * VD2_RATIO);
      Serial.println(" VAR");

      Serial.print("Power factor: ");
      Serial.println(powerFactor);
    break;

    default:
      Serial.println("/// SELECTED MODE ERROR ///");
    break;
  }
}
