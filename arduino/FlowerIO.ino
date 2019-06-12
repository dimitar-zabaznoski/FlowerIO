/**
   A self wattering plant system.

   A value is read from a moisture sensor and if it's below a threshold
   a pump is activated for PUMP_OP_CYCLE_TIME to water the plant.
   The system waits a COOLDOWN_TIME to allow the plant to absorb
   the watter and stabilize before checking again.
   As a fault prevention mechanism, if the plant is still dry after
   FAULT_THRESHOLD attempts the system enters error mode where it does
   nothing but bink a red LED.

   The system also listens for commands and sends messages on sowtware
   and hardware serial I/O. The received commands are echoed back as ACK.
   List of supported commands:
      "info_pump"       - Sends the pump run log as comma separated timestamp values in seconds;
      "info_moisture"   - Sends the MIN, MAX, and AVG readings from the moisture sensor;
      "manual_run"      - Initiates a manual start of the pump as if triggered from the sensor;
      "reset"           - Resets the fault count and manual run flags;
      "stop"            - Stops the pump and enteres error mode;

   The circuit:
    - RX is digital pin 10 (connect to TX of other device)
    - TX is digital pin 11 (connect to RX of other device)
    - PIN_9 used for AT command mode of HC-05 bluetooth module
    - PIN_12 used for blinking red LED error output
    - PIN_13 (LED_BUILTIN) used as pump running indicator
    - PIN_A0 used for analog read from the moisture sensor
    - PIN_9 (PWM) used to run the pump

   Uses 25% of program storage and 33% of dynamic memory (ArduinoUno)

   Note on serial communication:
   _____________________________
   Arduino reads the serial connection either byte by byte
   or tries to parse ASCII encoded integer or sting;
   [Make a diference between write and print & read and readString]

   @see the reference for software and hardware serial
   https://www.arduino.cc/en/Reference/Serial
   https://www.arduino.cc/en/Reference/SoftwareSerial

   @see AT command mode tutorial
   http://www.instructables.com/id/AT-command-mode-of-HC-05-Bluetooth-module/step5/AT-commands/

   @see Control DC Motors with L293D tutorial
   https://lastminuteengineers.com/l293d-dc-motor-arduino-tutorial/
*/
#include <SoftwareSerial.h>

#define PIN_ERROR 12
#define PIN_RX_SOFT 10
#define PIN_TX_SOFT 11
#define PIN_ANALOG_MOISTURE A0
#define PIN_DC_PUMP 9

/**
   Max logged pump run entries. After overflow the values are overwritten.
 */
#define PUMP_LOG_SIZE 20

/**
   Baud rate for the serial communication.
   Any standard rate can be set.
*/
#define BAUD_RATE 38400

/**
   Max tries to run the pump and water the plant.
   If the moisture sensor does not report a satisfactory value after the
   defined attempts the system enters an error mode and does not operate.

   Used to prevent the system from entering a loop when there is a fault:
   - No water in tank [running the pump without watter will cause damage]
   - The watter flow not reaching the plant [this can cause water leaks]
   - A fault in the moisture sensor [this will drown the plant and cause an overflow]
*/
const byte FAULT_THRESHOLD = 3;

/**
   The min value read from the moisture sensor that indicates that the plant is watered.
   This sensor reports low values for moist soil, and high values for dry soil.

   Value range: 0 - 1023
*/
const int MOISTURE_THRESHOLD = 300;

/**
   Time the system waits between subsequent moisture checks.
   We allow the plant to absorb the watter and stabilize during this time.
*/
const unsigned long COOLDOWN_TIME_SECONDS = 20;

/**
   When the pump runs, it operates in cycles that last a fixed time interval defined with this constant.
*/
const unsigned long PUMP_OP_CYCLE_TIME_SECONDS = 10;

//PWM values (for the DC pump)
const byte PWM_OFF = 0;
const byte PWM_25 = 64;
const byte PWM_50 = 127;
const byte PWM_75 = 191;
const byte PWM_MAX = 255;

//The PWM value used for running the pump
const byte DC_PUMP_STRENGTH = PWM_25;

//Pump
bool pumpRunning;
bool manualRun;
unsigned long pumpRunLogs[PUMP_LOG_SIZE]; //in seconds
unsigned long lastPumpRunPlusCooldown;
unsigned long lastPumpRunPlusOpTime;
int pumpLogIndex;
int index;


//Moisture
int moistureLastValue;
int moistureHigh;
int moistureLow;
unsigned long moistureValueReadCount; //max times this value can be read and have a valid average calucaltion is 4,199,426
unsigned long moistureValueReadSum; //the sum of all moisture readings used to calculate an average

//Util
unsigned long ellapsedTimeSeconds;
byte faultCount;
String serialMessageIn;


SoftwareSerial softSerial(PIN_RX_SOFT, PIN_TX_SOFT);

void setup() {
  pinMode(PIN_ERROR, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  faultCount = 0;
  pumpLogIndex = -1;
  moistureLastValue = -1;
  moistureHigh = -1;
  moistureLow = -1;
  serialMessageIn = "";
  for (index = 0; index < PUMP_LOG_SIZE; index++) {
    pumpRunLogs[index] = -1;
  }

  softSerial.begin(BAUD_RATE);
  Serial.begin(BAUD_RATE);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
}

void loop() {
  ellapsedTimeSeconds = millis() / 1000;

  //Check for commands (echo them back for ACK)
  if (softSerial.available()) {
    serialMessageIn = softSerial.readStringUntil('\n');
    Serial.println(serialMessageIn);
    softSerial.println(serialMessageIn);
  }
  if (Serial.available()) {
    serialMessageIn = Serial.readStringUntil('\n');
    Serial.println(serialMessageIn);
    softSerial.println(serialMessageIn);
  }
  if (serialMessageIn != "") {
    handleInputCommand(serialMessageIn);
    serialMessageIn = "";
  }

  //Read, log, and print moisture value
  moistureLastValue = analogRead(PIN_ANALOG_MOISTURE);
  // printValue("moisture = ", moistureLastValue);
  moistureValueReadSum += moistureLastValue;
  moistureValueReadCount++;
  if (moistureLow == -1 || moistureHigh == -1) {
    moistureLow = moistureLastValue;
    moistureHigh = moistureLastValue;
  } else if (moistureLastValue > moistureHigh) {
    moistureHigh = moistureLastValue;
  } else if (moistureLastValue < moistureLow) {
    moistureLow = moistureLastValue;
  }

  //Check if program is in ERROR_MODE
  if (faultCount >= FAULT_THRESHOLD) {
    if (pumpRunning) {
      runPumpWithLog(PWM_OFF);
      printValue("ERROR MODE - Turning pump off", -1);
    }
    digitalWrite(PIN_ERROR, HIGH);
    delay(1000);
    digitalWrite(PIN_ERROR, LOW);
    delay(1000);
    return;
  }

  //Check if pump needs to be turned off
  if (pumpRunning) {
    lastPumpRunPlusOpTime = pumpRunLogs[pumpLogIndex] + PUMP_OP_CYCLE_TIME_SECONDS;
    if(ellapsedTimeSeconds >= lastPumpRunPlusOpTime) {
      runPumpWithLog(PWM_OFF);
      printValue("Turning pump off at moisture value: ", moistureLastValue);
    }
  }

  //Check if manual run was requested
  if (manualRun) {
    runPumpWithLog(DC_PUMP_STRENGTH);
    manualRun = false;
  }

  //React to moisture sensor value (run or stop pump)
  if (pumpLogIndex != -1) {
    lastPumpRunPlusCooldown = pumpRunLogs[pumpLogIndex] + COOLDOWN_TIME_SECONDS;
  } else {
    lastPumpRunPlusCooldown = COOLDOWN_TIME_SECONDS;
  }
  if (moistureLastValue != -1 && moistureLastValue > MOISTURE_THRESHOLD) {
    if (!pumpRunning && ellapsedTimeSeconds >= lastPumpRunPlusCooldown) {
      faultCount++;
      runPumpWithLog(DC_PUMP_STRENGTH);
    }
  } else {
    faultCount = 0;
    if (pumpRunning && moistureLastValue > MOISTURE_THRESHOLD + 200) {
      runPumpWithLog(PWM_OFF);
      printValue("Turning pump off - max moisture reached before pump cycle at ellapsed time (s)", ellapsedTimeSeconds);
    }
  }

  delay(1000); //Delay for stability (min 1s)
}

/**
   Turn the pump off or on with defined strength (controlled with PWM).
   When the pump is running the built-in LED is turned on.

   Important: All pump runs are logged. This value is then used to
   automatically turn the pump off when conditions are met.
*/
void runPumpWithLog(byte strength) {
  analogWrite(PIN_DC_PUMP, strength);

  if (strength == PWM_OFF) {
    digitalWrite(LED_BUILTIN, LOW);
    pumpRunning = false;
  } else {
    digitalWrite(LED_BUILTIN, HIGH);
    pumpRunning = true;

    //Log pump run
    pumpLogIndex++;
    if (pumpLogIndex == PUMP_LOG_SIZE) {
      pumpLogIndex = 0;
    }
    pumpRunLogs[pumpLogIndex] = ellapsedTimeSeconds;
    printValue("pumpRunLog(seconds)=", ellapsedTimeSeconds);
  }
}

/**
   Prints a value to hardware and software serial outputs.
   The message is concatenations of params keyAndOperator and value.
*/
void printValue(String keyAndOperator, long value) {
  String message;
  if (value != -1) {
    message = keyAndOperator + value;
  } else {
    message = keyAndOperator;
  }
  Serial.println(message);
  softSerial.println(message);
}

/**
   Handles input commands from sofware and hardware serial communication.
   For a description of each command see the top javadoc.
*/
void handleInputCommand(String command) {
  if (command == "info_pump") {
    if (pumpLogIndex != -1) {
      String commaSeparatedLogs = "";
      for (index = pumpLogIndex; index >= 0; index--) {
        if (pumpRunLogs[index] != -1) {
          commaSeparatedLogs += pumpRunLogs[index];
          commaSeparatedLogs += ",";
        }
      }
      for (index = PUMP_LOG_SIZE - 1; index > pumpLogIndex; index--) {
        if (pumpRunLogs[index] != -1) {
          commaSeparatedLogs += pumpRunLogs[index];
          commaSeparatedLogs += ",";
        }
      }
      printValue(commaSeparatedLogs, -1);
    } else {
      printValue("No wattering logs", -1);
    }
  } else if (command == "info_moisture") {
    printValue("Moisture current: ", moistureLastValue);
    printValue("Moisture high: ", moistureHigh);
    printValue("Moisture low: ", moistureLow);
    if (moistureValueReadCount > 0) {
      unsigned long moistureAvg = moistureValueReadSum / moistureValueReadCount;
      printValue("Moisture avg.: ", moistureAvg);
    } else {
      printValue("Moisture avg.: ", -1);
    }
  } else if (command == "manual_run") {
    if (!manualRun && !pumpRunning) {
      manualRun = true;
    }
  } else if (command == "reset") {
    manualRun = false;
    faultCount = 0;
  } else if (command == "stop") {
    manualRun = false;
    faultCount = FAULT_THRESHOLD;
  } else {
    String message = "Command not recognized: " + command;
    printValue(message, -1);
  }
}
