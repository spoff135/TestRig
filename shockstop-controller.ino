// Regulator 1/DAC 1 is pull (down), Pressure Sensor 1
// Regulator 2/DAC 2 is push (up), Pressure Sensor 0

int version = 6142; // version number (month.day.rev)
int testMode = 5; // current mode (0=no test running,1=in-phase test,2=out-of-phase test,3=realworld in-phase, 5=elastomer testing)
int cycleCount = 0;
int cycleTarget = 100000;
int stateTimeout[3] = {2000,2000,2000}; // time (ms) before automatic state change
int minStateTime = 200; // minimum amount of time before test fixture looks to see if state has reached desired pressure
int measuredForceBuffer = 5;
float windowExcursionLimit = 1.1;
bool webUpdateFlag = true;
bool errorChecking = true;
int dashboardRefreshRate = 5000; // dashboard refresh rate (ms)
int testNumber = 1; // automatically incremented with each test

//UBIDOTS CODE
#include "HttpClient.h"  // if using webIDE, use this: #include "HttpClient/HttpClient.h"
#define WEB_DEFLECTION_AVG "56b671fb76254228866ee416"
#define WEB_DEFLECTION "56cc63b6762542644cc8d2ef"
#define WEB_CYCLES "56b672cb7625422dd8dbbf52"
#define WEB_COMPRESSOR_STATE "56cc62f37625425f3cd6aeb2"
#define WEB_FORCE_UP "56cc62a27625425dba666c80"
#define WEB_FORCE_DOWN "56cc666d7625427368e80d6e"
#define WEB_FORCE_INPUT "56cc62db7625425f3cd6ae79"

HttpClient http;
// Headers currently need to be set at init, useful for API keys etc.
http_header_t headers[] = {
    { "Content-Type", "application/json" },
    { "X-Auth-Token: Me72mxyaIELmctKM81Y0DVY3MTUJ4z" },
    { NULL, NULL } // NOTE: Always terminate headers with NULL
};

http_request_t request;
http_response_t response;
//UBIDOTS CODE

int LCDrefreshRate = 300; //LCD refresh rate (ms)

// Define digital pins
int leftUp = D2; // relay 1
int rightUp = D3; // relay 2
int leftDown = D4; // relay 3
int rightDown = D5; // relay 4
int displayTogglePin = D6; // toggle switch for display
int compressorRelayPin = D7; // pin for compressor on/off relay
int testRelayPin = 0; // updated by calibrate function

// Define analog pins
int muxSelectorPin = WKP; // analog pin (used as digital) associated with
int pressurePosPin = A4; //analog pin associated with pressure sensor Vout+
int pressureNegPin = A5; //analog pin associated with pressure sensor Vout-
int leftDucerPin = A2; //analog pin associated with the left transducer
int rightDucerPin = A1; //analog pin associated with the right transducer

// Define system hardware constants
float pullArea = 2.8348; // mcm part 6498K477
float pushArea = 3.1416; // mcm part 6498K477
//float pullArea = 4.6019; // mcm part 6498K488
//float pushArea = 4.9087; // mcm part 6498K488
int minPSI = 3; // regulator min pressure
int maxPSI = 120; // regulator max pressure
int maxResolution = 4095;
int maxTestLoad_Elastomer = 200; // maximum test force used in GenerateElastomerFvD
int maxTestLoad_Bar = 120; // maximum force (per cylinder) used in GenerateFvD
    // Define I2C addresses
int calibrateTwistLoad = 65; // maximum test force used in calibrateTwist
int pressureSensorAddress = 40;//same as 0x28
int dataLoggerAddress = 8; // address of dataLogger
    // Regulator calibration settings
int intercept100psi = 3550;
int intercept10psi = 515;
int intercept3psi = 250;
    // Pressure Sensor Constants
byte mask = 63; // 00111111
int countsMax = 14745; //bits
int countsMin = 1638; //bits
int sensorMax = 150; //psi
int sensorMin = 0; //psi
    // Compressor limits
int tankMin = 65; // tank pressure at which the compressor turns on
int tankMax = 90; // tank pressure at which the compressor turns off
    // Define LCD Constants
char ESC = 0xFE;

// Initialize system/state variables
    // count variables
int nStates = 0; // number of states in the testMode
int i = 0;
int dashboardUpdateCount = 0;
int errorCountConsecutive = 0; // count of consecutive errors
int errorLog = 0; // total count of all errors (including non-consecutive)
int webUpdateCount = 0;
int pressureSensorSelector = 0; // 0 = P1 (pull), 1 = P2 (push)
    // timing variables
long timeLeft = 0; // estimated time remaining to reach cycleTarget
long lastLCDupdate = 0; //time of last LCD update
long lastI2Cupdate = 0; // time of last I2C status check
long lastDashboardUpdate = 0; // time of last dashboard update
long compressorStartTime = 0; // time in ms that compressor was turned on
long compressorStopTime = 0; // time in ms that compressor was turned off
long compressorOnTime = 0; // # of ms compressor was on
long compressorOffTime = 0; // # of ms compressor was off
long stateStartTime = 0; // start time (ms) of current state
long stateTime = 0; // elapsed time (ms) in current state
    // outputs state flags/variables
bool paused = true;
bool statusUpdate = true;
bool useI2C = true;
bool usePressureGauge = false;
bool compressorOn = false;
int currentState = 0;  // current state (0=null,1 & 2 are mode-dependent)
int DAC1_bits = 0; // current setting of DAC1
int DAC2_bits = 0; // current setting of DAC2
float PSI1setting = 0; // current setting of DAC1 in PSI
float PSI2setting = 0; // current setting of DAC2 in PSI
float forceSetting = 0; // current force setting
String errorMsg = "";
    // Measured state flags/variables
bool dataLoggerStatus = false;
bool pressureSensorStatus = false;
bool errorFlag = false;
int displayMode = 0; // can be mode 0, 1 or 2
int disp1 = 0; // display mode modifier
int leftDucerPosBits = 0; //value used to store voltage in bits
int rightDucerPosBits = 0; //value used to store voltage in bits
    // Calculated state values
float tankPressurePSI = 80; // value used to store tank pressure in PSI
float pushPressurePSI = 0; // value used to store right cylinder push pressure in PSI
float pullPressurePSI = 0; // value used to store left cylinder push pressure in PSI
float state1Force = 0; // updated in CheckStateConditions
float state2Force = 0; // updated in CheckStateConditions
float measuredPushForce = 0; // = pushPressurePSI * area in^2
float measuredPullForce = 0; // = pullPressurePSI * area in^2
float leftDucerPosInch = 0; // value used to store position in inches from zero position
float rightDucerPosInch = 0; // value used to store position in inches from zero position
float compressorDutyCycle = 0.5;
    // Measured/calculated values related to position
float positionLeft[3] = {0,0,0}; // position at end of state [i]
float positionRight[3] = {0,0,0};
float refPositionLeft[3] = {0,0,0}; // reference position from calibration of state [i]
float refPositionRight[3] = {0,0,0};
float refPositionLeftWindow[3] = {0,0,0}; // max deviation allowed from reference position of state [i]
float refPositionRightWindow[3] = {0,0,0};
float deflection; // Total deflection
float deflectionAvg; // deflection running average
float refDeflection; // Total deflection measured during most recent calibration
float deflectionMax; // Total deflection limit
float lastNeutralLeft; // Most recent recorded neutral position
float lastNeutralRight; // Most recent recorded neutral position

// Flags for performing functions
bool testRelay = false;
bool testRelays = false;
bool testCylinder = false;
bool calibrateInPhase = false;
bool calibrateOutOfPhase = false;
bool calibratePressure = false;
bool calibrateWindow = false;
bool generateFvD = false; // generates force vs displacement graph
bool elastomerFvD = false; // generates force vs displacement graph for elastomers
bool iterateFvD = false; // flag for iterative FvD graph
bool continuousFvD = false; // flag for continuous force vs displacement graph for elastomers

void setup()
{
    Wire.begin(); // join i2c bus (address optional for master)
    Serial1.begin(9600); // Initialize serial line for LCD
    InitializeLCD(); // Initialize LCD

    // Configure digital pins as input or output
    pinMode(compressorRelayPin, OUTPUT);
    pinMode(leftUp, OUTPUT);
    pinMode(rightUp, OUTPUT);
    pinMode(leftDown, OUTPUT);
    pinMode(rightDown, OUTPUT);
    pinMode(displayTogglePin, INPUT);

    // Declare a Particle.functions so that we can access them from the cloud.
    Particle.function("psi",WebTestPressure);
    Particle.function("force",WebSetForce);
    Particle.function("run",WebRunFunction);
    Particle.function("timeout",WebSetTimeout);

    // Initialize analog outputs to zero
    pinMode(DAC1, OUTPUT);
    analogWrite(DAC1,DAC1_bits);
    pinMode(DAC2, OUTPUT);
    analogWrite(DAC2,DAC2_bits);
    pinMode(muxSelectorPin, OUTPUT);
    analogWrite(muxSelectorPin,0);
    pinMode(leftDucerPin, INPUT);
    pinMode(rightDucerPin, INPUT);

    // Ensure no relays are open and pressure is at 0
    KillAll();
    SetPressure(0,1);
    SetPressure(0,2);
    forceSetting = 0;
    delay(500);

    // Initialize neutral values
    ReadInputPins();
    compressorStartTime = millis();
    compressorStopTime = millis();

    //UBIDOTS CODE
    request.hostname = "things.ubidots.com";
    request.port = 80;

}// Setup


void loop()
{
    ReadInputPins();
    RunCalibrations();// runs functions enabled through webhooks

    bool stateChange = false;
    currentState = 1; // start in state 1.
    SetState(currentState);

    if(cycleCount>=cycleTarget) paused=true; // pause test if cycleTarget is reached

    while(!paused && currentState<=nStates && testMode>0){
        ReadInputPins();
        stateChange = CheckStateConditions(currentState);
        if(stateChange){

            // Before changing state, update positions
            positionLeft[currentState] = leftDucerPosInch;
            positionRight[currentState] = rightDucerPosInch;

            if(!paused){
                currentState = currentState+1;
                SetState(currentState);
                stateStartTime = millis();
            }
            else break;
        }
        stateTime = millis()-stateStartTime;

        PrintStatusToLCD("Run");

        //delay(1);//tbd moved to ReadInputPins
    }

    // Update deflection total and average and check against window
    deflection = CalculateDeflection();
    deflectionAvg = 0.9*deflectionAvg + 0.1*deflection;
    if(deflection > deflectionMax){
        errorFlag = true;
        errorMsg = "Total defl error";
    }
    for(int i=1; i<=nStates; i++){
        if(abs(positionLeft[i]-refPositionLeft[i]) > refPositionLeftWindow[i]){
            errorFlag = true;
            errorMsg = "Error: LeftPos, S" + String(i);
        }
        else if(abs(positionRight[i]-refPositionRight[i]) > refPositionRightWindow[i]){
            errorFlag = true;
            errorMsg = "Error: RightPos, S" + String(i);
        }
    }

    // This allows only one error to count per cycle
    if(errorFlag){
        errorCountConsecutive++;
        errorLog++;
        errorFlag = false;
    }
    else errorCountConsecutive = 0;

    // Only error out if there have been 2 consecutive errors
    if(errorCountConsecutive > 1 && errorChecking){
        paused = true;
        errorFlag = true;
    }

    PrintStatusToLCD("Run");

    if(webUpdateFlag) UpdateDashboard();

    if(paused){
        PauseAll();
    }

}// loop


//------------------------------------------------------------------------
bool CheckStateConditions(int currentState){
    bool stateChange = false;

    switch (currentState) {
    case 1:
        if(stateTime > stateTimeout[currentState]) stateChange = true;
        if(usePressureGauge && stateTime > minStateTime && measuredPushForce > (forceSetting + measuredForceBuffer) ) stateChange = true;
        if(stateChange){
          state1Force = measuredPushForce;
          if(state1Force < forceSetting){
            PSI2setting+=0.1;
            SetPressure(PSI2setting,2);
          }
          else if(state1Force > forceSetting){
            PSI2setting-=0.1;
            SetPressure(PSI2setting,2);
          }

        }
        break;
    case 2:
        if(stateTime > stateTimeout[currentState]) stateChange = true;
        if(usePressureGauge && stateTime > minStateTime && measuredPullForce > (forceSetting + measuredForceBuffer) ) stateChange = true;
        if(stateChange){
          state2Force = measuredPullForce;
          if(state2Force < forceSetting){
            PSI1setting+=0.1;
            SetPressure(PSI1setting,1);
          }
          else if(state2Force > forceSetting){
            PSI1setting-=0.1;
            SetPressure(PSI1setting,1);
          }

        }
        break;
    case 3:
        if(stateTime > stateTimeout[currentState]) stateChange = true;
        break;
    case 4:
        if(stateTime > stateTimeout[currentState]) stateChange = true;
        break;
    }

    return stateChange;
}// CheckStateConditions


//------------------------------------------------------------------------
void SetState(int currentState){
    if(testMode==1){ // in-phase ISO test
        switch (currentState) {
        case 1:
            PushUp();
            break;
        case 2:
            PushDown();
            cycleCount++;
            break;
        }
    }
    else if(testMode==2){ // out-of-phase ISO test
        switch (currentState) {
        case 1:
            TwistLeft();
            break;
        case 2:
            TwistRight();
            cycleCount++;
            break;
        }
    }
    else if(testMode==3){ // real-world in-phase test
        switch (currentState) {
        case 1:
            KillRelays();
            break;
        case 2:
            PushDown();
            cycleCount++;
            break;
        }
    }
// test mode 4 doesn't work because deflection is only calculated based on 2 states
/*    else if(testMode==4){ // real-world out-of-phase test
        switch (currentState) {
        case 1:
            KillRelays();
            break;
        case 2:
            LeftDown();
            break;
        case 3:
            KillRelays();
            break;
        case 4:
            RightDown();
            cycleCount++;
            break;
        }
    }
*/
    else if(testMode==5){ // elastomer test fixture (right cylinder only)
        switch (currentState) {
        case 1:
            KillRelays();
            break;
        case 2:
            RightDown();
            cycleCount++;
            break;
        }
    }
    else {
//        KillAll();
    }
}// SetState


//------------------------------------------------------------------------
void SetMode(int testMode){
    switch (testMode)
    {
    case 0:
        testMode = 0;
        nStates = 0;
        break;

    case 1:  // in-phase ISO test
        testMode = 1;
        nStates = 2;
        break;

    case 2:  // out-of-phase ISO test
        testMode = 2;
        nStates = 2;
        break;

    case 3:  // in-phase real world test
        testMode = 3;
        nStates = 2;
        break;
// test mode 4 doesn't work because deflection is only calculated based on 2 states
/*
    case 4:  // out-of-phase real world test
        testMode = 4;
        nStates = 4;
        break;
*/
    case 5: // elastomer test fixture (right cylinder only)
        testMode = 5;
        nStates = 2;
        break;
    default:
        testMode = 0;
        nStates = 0;
        KillAll();
        return; // return before setting calibrate window flag
        break;
    }

    calibrateWindow = true;
}// SetMode


//------------------------------------------------------------------------
float CalculateDeflection(){

    float leftDef = positionLeft[2]-positionLeft[1];
    if(leftDef < 0) leftDef = leftDef*-1;

    float rightDef = positionRight[2]-positionRight[1];
    if(rightDef < 0) rightDef = rightDef*-1;

    float totalDef = leftDef + rightDef;
    return totalDef;
}// CalculateDeflection


//------------------------------------------------------------------------
void InitializeLCD(){
    // Initialize LCD module
    Serial1.write(ESC);
    Serial1.write(0x41);
    Serial1.write(ESC);
    Serial1.write(0x51);
    // Set Contrast
    Serial1.write(ESC);
    Serial1.write(0x52);
    Serial1.write(40);
    // Set Backlight
    Serial1.write(ESC);
    Serial1.write(0x53);
    Serial1.write(8);

    ClearLCD();
    lastLCDupdate = millis();
}// InitializeLCD


//------------------------------------------------------------------------
void ClearLCD(){
    Serial1.write(ESC);
    Serial1.write(0x51);
}// ClearLCD


//------------------------------------------------------------------------
void WriteLineToLCD(String msg, int row){
    // Place cursor on correct row
    Serial1.write(ESC);
    Serial1.write(0x45);
    if(row==1) Serial1.write(0x00);
    else if(row==2) Serial1.write(0x40);
    else if(row==3) Serial1.write(0x14);
    else if(row==4) Serial1.write(0x54);
    else return;

    // Write only 20 characters
    Serial1.write(msg.substring(0,20));
}// WriteLineToLCD


//------------------------------------------------------------------------
// Reads input pins and updates the appropriate variables
void ReadInputPins(){

    // Check to see if I2C hardware is responding
    if(useI2C && millis()-lastI2Cupdate > LCDrefreshRate){
        dataLoggerStatus = CheckI2C(dataLoggerAddress);
        pressureSensorStatus = CheckI2C(pressureSensorAddress);
        lastI2Cupdate = millis();
    }

    pushPressurePSI = ReadDigitalPressureSensor(1);
    measuredPushForce = pushPressurePSI * pushArea;
    pullPressurePSI = ReadDigitalPressureSensor(0);
    measuredPullForce = pullPressurePSI * pullArea;

    // Check tank pressure and turn on/off as necessary
    tankPressurePSI = 0.1 * ReadAnalogPressureSensor() + (0.9) * tankPressurePSI; // poor man's running average (alpha factor of 0.1)

    if(tankPressurePSI < tankMin && !compressorOn){
        digitalWrite(compressorRelayPin,HIGH);
        compressorOn = true;
        compressorStartTime = millis();
        compressorOffTime = compressorStopTime - compressorStartTime;
    }
    else if(tankPressurePSI > tankMax && compressorOn){
        digitalWrite(compressorRelayPin,LOW);
        compressorOn = false;
        compressorStopTime = millis();
        compressorOnTime = compressorStartTime - compressorStopTime;
        compressorDutyCycle = 0.7 * (compressorOnTime) / (compressorOnTime + compressorOffTime) + 0.3 * compressorDutyCycle;
    }

    // Read transducer (position) pins
    leftDucerPosBits = analogRead(leftDucerPin);
    rightDucerPosBits = analogRead(rightDucerPin);

    leftDucerPosInch = leftDucerPosBits * 0.00096118; // based on 100mm ducer/4096 bits = .00096118"/bit
    rightDucerPosInch = rightDucerPosBits * 0.00096118; // based on 100mm ducer/4096 bits = .00096118"/bit

    // Read Display Switch
    displayMode = digitalRead(displayTogglePin) + disp1;

    // update timeLeft
    timeLeft =  ( (cycleTarget-cycleCount) * (stateTimeout[currentState]+stateTimeout[currentState])/2.0 * nStates ) / 60000.0;
}// ReadInputPins


//------------------------------------------------------------------------
void RunCalibrations(){
    if(calibrateInPhase){
        CycleInPhase(5,30);
        calibrateInPhase = false;
    }

    if(calibrateOutOfPhase){
        for(i=5; i<=calibrateTwistLoad; i+=5)
        CycleOutOfPhase(5,i);
        calibrateOutOfPhase = false;
    }

    if(calibratePressure){
        CalibratePressure();
        calibratePressure = false;
    }

    if(generateFvD){
        GenerateFvD();
        generateFvD = false;
    }

    if(elastomerFvD){
        GenerateElastomerFvD();
        elastomerFvD = false;
    }

    if(continuousFvD){
        GenerateContinuousFvD();
        continuousFvD = false;
    }

    if(testRelays){
        TestRelays();
        testRelays = false;
    }

    if(testRelay){
        digitalWrite(testRelayPin,HIGH);
        delay(stateTimeout[0]);
        digitalWrite(testRelayPin,LOW);
        testRelay = false;
    }

    if(calibrateWindow){
        ResetNeutral();

        // run through each state and record the final deflection to set deflectionWindow
        int i;
        for(i=0;i<=nStates;i++){
            PrintStatusToLCD("Calibrate State " + String(i));
            SetState(i);
            delay(stateTimeout[i]);
            ReadInputPins();
            refPositionLeft[i] = leftDucerPosInch;
            refPositionRight[i] = rightDucerPosInch;

            positionLeft[i] = leftDucerPosInch; // added these so "CalculateDeflection" has something to reference.
            positionRight[i] = rightDucerPosInch;
        }

        // update reference deflection
        refDeflection = CalculateDeflection();
        deflectionMax = refDeflection * windowExcursionLimit;

        // Update deflection windows based on the current mode
        for(i=1;i<=nStates;i++){
            refPositionLeftWindow[i] = (refDeflection*(windowExcursionLimit-1));
            refPositionRightWindow[i] = (refDeflection*(windowExcursionLimit-1));
        }

        calibrateWindow = false;
        PrintStatusToLCD("");
        ResetNeutral();
        delay(2000);
    }
}// RunCalibrations


//------------------------------------------------------------------------
// Prints measured and state variables to i2c & LCD
int PrintStatus(String msg){
    msg += "," + String(pullPressurePSI,2);
    msg += "," + String(leftDucerPosInch,2);
    msg += "," + String(rightDucerPosInch,2);

    PrintMsg(msg);
    ClearLCD();
    Serial1.write(msg);

    return 1;
}// PrintStatus


//------------------------------------------------------------------------
// Prints measured and state variables to LCD
int PrintStatusToLCD(String origMsg){
    String msg = "";
    if(millis()-lastLCDupdate > LCDrefreshRate){
        if(displayMode==0){
            ClearLCD();

            msg = "Ver:" + String(version) + " E"+ String(errorCountConsecutive) + " M" + String(testMode);
            msg += "                "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
            WriteLineToLCD(msg,1);

            msg = "#" + String(cycleCount) + "/" + String(cycleTarget);
            msg += " " + String(timeLeft) + "min";
            msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
            WriteLineToLCD(msg,2);

            if(useI2C) msg = origMsg + " | D:" + String(dataLoggerStatus) + " S:" + String(currentState) + " T:" + String(testNumber);
            else msg = origMsg + " | I2C Off";
            msg += "         ";
            WriteLineToLCD(msg,3);

            msg = "Tank:" + String(tankMin);
            msg += "<" + String(tankPressurePSI,1);
            msg += "<" + String(tankMax);
            msg += "|" + String(compressorDutyCycle,1);
            msg += "         ";
            WriteLineToLCD(msg,4);

//            msg = "L" + String(leftDucerPosInch,3);
//            msg += " R" + String(rightDucerPosInch,3);
//            msg += "         ";WriteLineToLCD(msg,4);

            lastLCDupdate = millis();
        }
        else if(displayMode==1){
            msg = "M" + String(testMode);
            msg += "/" + String(refDeflection,3);
            msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
            WriteLineToLCD(msg,1);

            msg = "Left:" + String(positionLeft[1],2);
            msg += "/" + String(positionLeft[2],2);
            msg += "/" + String(refPositionLeft[2],2);
            msg += "         ";
            WriteLineToLCD(msg,2);

            msg = "Right:" + String(positionRight[1],2);
            msg += "/" + String(positionRight[2],2);
            msg += "/" + String(refPositionRight[2],2);
            msg += "         ";
            WriteLineToLCD(msg,3);

            msg = "D:" + String(deflection,3);
            msg += "/" + String(deflectionAvg,3);
            msg += "/" + String(deflectionMax,3);
            msg += "         ";
            WriteLineToLCD(msg,4);

            lastLCDupdate = millis();
        }
        else if(displayMode==2){
            msg = origMsg;
            msg += "Ver:" + String(version);
            msg += " Fs:" + String(forceSetting,0);
            msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
            WriteLineToLCD(msg,1);

            msg = "Pll:" + String(measuredPullForce,1);
            msg += " Ps" + String(PSI1setting,1);
            msg += " P" + String(pullPressurePSI,1);
            msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
            WriteLineToLCD(msg,2);

            msg = "Psh:" + String(measuredPushForce,1);
            msg += " Ps" + String(PSI2setting,1);
            msg += " P" + String(pushPressurePSI,1);
            msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
            WriteLineToLCD(msg,3);

            msg = "Defl " + String(deflection,2);
            msg += "/" + String(deflectionAvg,3);
            msg += "/" + String(deflectionMax,3);
            msg += "         ";
            WriteLineToLCD(msg,4);

            lastLCDupdate = millis();
        }

    }
    return 1;
}// PrintStatusToLCD


//------------------------------------------------------------------------
// Prints msg to i2c
int PrintMsg(String msg){

    if(useI2C){
        String msgChunk = "";
        while(strlen(msg) > 0){ //transmit in 32 byte chunks
            msgChunk = msg.substring(0,31); // extract 32 byte chunk
            Wire.beginTransmission(dataLoggerAddress); // transmit to dataLoggerAddress
            Wire.write(msgChunk);        // send chunk
            Wire.endTransmission();    // stop transmitting
            msg.remove(0,31);           // remove chunk
            delay(2); // prevents lost data? tbd
        }

        // send end of line character
        Wire.beginTransmission(dataLoggerAddress);
        Wire.write((byte)0x0D);
        Wire.endTransmission();
    }
    return 1;
}// PrintMsg

//------------------------------------------------------------------------
void UpdateDashboard(){

    if(millis()-lastDashboardUpdate > dashboardRefreshRate){

      request.body = "[";
      request.body += "{\"variable\":\""WEB_CYCLES"\", \"value\": "+String(cycleCount)+" }";
      request.body += ", { \"variable\":\""WEB_DEFLECTION"\", \"value\": "+String(deflection,3)+" }";
      request.body += ", { \"variable\":\""WEB_DEFLECTION_AVG"\", \"value\": "+String(deflectionAvg,3)+" }";
      request.body += ", { \"variable\":\""WEB_FORCE_UP"\", \"value\": "+String(state1Force,1)+" }";
      request.body += ", { \"variable\":\""WEB_FORCE_DOWN"\", \"value\": "+String(state2Force,1)+" }";
      request.body += ", { \"variable\":\""WEB_FORCE_INPUT"\", \"value\": "+String(forceSetting)+" }";
      request.body += ", { \"variable\":\""WEB_COMPRESSOR_STATE"\", \"value\": "+String(compressorOn)+" }";
            request.body += "]";
      request.path = "/api/v1.6/collections/values/";
      http.post(request, response, headers);

/*
      request.body = "[";
      request.body += "{\"variable\":\""WEB_CYCLES"\", \"value\": 2}";
      request.body += ", { \"variable\":\""WEB_DEFLECTION"\", \"value\": 2}";
      request.body += ", { \"variable\":\""WEB_DEFLECTION_AVG"\", \"value\": 2}";
      request.body += ", { \"variable\":\""WEB_FORCE"\", \"value\": 2}";
      request.body += ", { \"variable\":\""WEB_FORCE_INPUT"\", \"value\": 2}";
      request.body += ", { \"variable\":\""WEB_COMPRESSOR_STATE"\", \"value\": 2}";
      request.body += "]";
      request.path = "/api/v1.6/collections/values/";
      http.post(request, response, headers);
*/

/*Working code
      // send deflection avg
      request.body = "{\"value\":" + String(deflectionAvg,3) + "}";
      request.path = "/api/v1.6/variables/"WEBDEFLECTION"/values?token="UBIDOTS_TOKEN;
*/
      lastDashboardUpdate = millis();
    }
}// UpdateDashboard


//------------------------------------------------------------------------
void PrintDiagnostic(String stateStr){
    // Message Header "Time,SystemState,cycleCount,testNumber,ForceSet,PSI1set,PSI2set,pullPressurePSI,pushPressurePSI,DAC1_bits,leftDucerPosBits,rightDucerPosBits,leftDucerPosInch,rightDucerPosInch,lastNeutralLeft,lastNeutralRight"
    String msg = "";

    msg += String(Time.now()) + ",";
    msg += stateStr + ",";
    msg += String(cycleCount) + ",";
    msg += String(testNumber) + ",";
    msg += String(forceSetting,2) + ",";
    msg += String(PSI1setting,2) + ","; //pull
    msg += String(PSI2setting,2) + ","; //push
    msg += String(pullPressurePSI,2) + ",";
    msg += String(pushPressurePSI,2) + ",";
    msg += String(DAC1_bits) + ",";
    msg += String(leftDucerPosBits) + ",";
    msg += String(rightDucerPosBits) + ",";
    msg += String(leftDucerPosInch,3) + ",";
    msg += String(rightDucerPosInch,3) + ",";
    msg += String(lastNeutralLeft,3) + ",";
    msg += String(lastNeutralRight,3);

    PrintMsg(msg);
}// PrintDiagnostic


//------------------------------------------------------------------------
void CycleInPhase(int numCycles, int force){
    int i = 0;
    int delayMS = 1000;

    PrintMsg("");
    PrintMsg("Running in-phase test for " + String(numCycles) + " cycles at " + String(force) + " lbs. on " + Time.timeStr());

    ResetNeutral();
    SetForce(force);
    delay(delayMS);
    ReadInputPins();
    PrintDiagnostic("Neutral");
    PrintStatusToLCD("Neutral");

    for(i=0; i<numCycles; i++){
        PushDown();
        delay(delayMS);
        ReadInputPins();
        PrintDiagnostic("Low");
        PrintStatusToLCD("Low");

    	PushUp();
        delay(delayMS);
        ReadInputPins();
    	PrintDiagnostic("High");
        PrintStatusToLCD("High");
    }

    ResetNeutral();
    delay(delayMS);
    ReadInputPins();
    PrintDiagnostic("Neutral");
    PrintMsg("Test Complete.");
}// CycleInPhase


//------------------------------------------------------------------------
void CycleOutOfPhase(int numCycles, int force){
    int i = 0;
    int delayMS = 1000;

    PrintMsg("");
    PrintMsg("Running out-of-phase test for " + String(numCycles) + " cycles at " + String(force) + " lbs. on " + Time.timeStr());

    ResetNeutral();
    SetForce(force);
    delay(delayMS);
    ReadInputPins();
    PrintDiagnostic("Neutral");
    PrintStatusToLCD("Neutral");

    for(i=0; i<numCycles; i++){
        TwistLeft();
        delay(delayMS);
        ReadInputPins();
        PrintDiagnostic("LeftDown");
        PrintStatusToLCD("LeftDown");

		TwistRight();
        delay(delayMS);
        ReadInputPins();
        PrintDiagnostic("LeftUp");
        PrintStatusToLCD("LeftUp");
    }

    ResetNeutral();
    delay(delayMS);
    ReadInputPins();
    PrintDiagnostic("Neutral");
    PrintMsg("Test Complete.");
}// CycleOutOfPhase


//------------------------------------------------------------------------
void CalibratePressure(){
    PrintMsg("Calibrating Pressure");

    int j =0;

    for(i=minPSI;i<100;i++){
        SetPressure(i,1);
        SetPressure(i,2);

        // delay and read inputs to smooth PSI reading
        for(j=0;j<100;j++){
            ReadInputPins();
            delay(10);
        }
        ReadInputPins();
        PrintDiagnostic("");
        PrintStatusToLCD("");
    }

    for(i=minPSI;i<100;i+=10){
        SetPressure(i,1);
        SetPressure(i,2);

        // delay and read inputs to smooth PSI reading
        for(j=0;j<400;j++){
            ReadInputPins();
            delay(10);
        }
        ReadInputPins();
        PrintDiagnostic("");
        PrintStatusToLCD("");
    }

    KillAll();
    PrintMsg("Pressure Calibration Complete");
}// CalibratePressure


//------------------------------------------------------------------------
void GenerateFvD(){

    int originalForceSetting = forceSetting; // record original force setting to reset to after test
    ResetNeutral();
    ReadInputPins();
    PrintDiagnostic("Neutral");
    PrintStatusToLCD("Neutral");

    for(i=10; i<=maxTestLoad_Bar; i+=1){
        if(i>20) i+=1;
        forceSetting = i;
        SetForce(i);
        PushDown();
        delay(1500);
        ReadInputPins();
        PrintDiagnostic("FvD "+ String(i));
        PrintStatusToLCD("FvD "+ String(i));
        ResetNeutral();
    }

    delay(500);
    ReadInputPins();
    PrintDiagnostic("Neutral");
    PrintMsg("FvD Test Complete.");

    // reset force setting
    forceSetting = originalForceSetting;
    SetForce(forceSetting);

    // increment test number
    testNumber++;
}// GenerateFvD

void GenerateContinuousFvD(){
  int originalForceSetting = forceSetting; // record original force setting to reset to after test

  SetForce(5);
  delay(250);
  for(i=0; i<3; i++){ // cycle max load down 3 times to "set" elastomer
    SetForce(maxTestLoad_Elastomer);
    RightDown();
    delay(1000);
    SetForce(5);
    RightUp();
    delay(2000);
  }
  delay(10000); // delay 10s to allow elastomer to recover

  // Record max up position
  SetForce(5);
  delay(250);
  RightUp();
  delay(1000);
  ReadInputPins();
  PrintDiagnostic("Up");
  PrintStatusToLCD("Up");

  // Record "neutral" position
  SetForce(0);
  RightDown();
  ReadInputPins();
  PrintDiagnostic("Neutral");
  PrintStatusToLCD("Neutral");
  delay(1000);

  int i = 0;
  RightDown();
  for(i=0; i<maxTestLoad_Elastomer; i++){
    forceSetting = i;
    SetForce(i);
    delay(50);
    ReadInputPins();
    PrintDiagnostic("FvD "+ String(i));
//    PrintStatusToLCD("FvD "+ String(i));
    delay(50);
  }

  // reset position by pushing up
  SetForce(5);
  PushUp();
  delay(1000);
  KillAll();

}

//------------------------------------------------------------------------
void GenerateElastomerFvD(){

    int originalForceSetting = forceSetting; // record original force setting to reset to after test

    SetForce(5);
    delay(250);
    for(i=0; i<3; i++){ // cycle max load down 3 times to "set" elastomer
      SetForce(maxTestLoad_Elastomer);
      RightDown();
      delay(1000);
      SetForce(5);
      RightUp();
      delay(2000);
    }
    delay(10000); // delay 10s to allow elastomer to recover

    // Record max up position
    SetForce(5);
    delay(250);
    PushUp();
    delay(1000);
    ReadInputPins();
    PrintDiagnostic("Up");
    PrintStatusToLCD("Up");

    // Record "neutral" position
    SetForce(0);
    PushDown();
    ReadInputPins();
    PrintDiagnostic("Neutral");
    PrintStatusToLCD("Neutral");

    int maxStart;
    int maxLoad;
    if(iterateFvD) maxStart = 0;
    else maxStart = maxTestLoad_Elastomer;

    // outer loop allows for iterativeFvD
    for(maxLoad = maxStart; maxLoad <= maxTestLoad_Elastomer; maxLoad+=10){
      // inner loop records force vs displacement from 0 to maxLoad
      for(i=0; i<=maxLoad; i+=2){
          if(i>16) i+=3; // after 16 lbs, increment by a total of 5
          forceSetting = i;
          SetForce(i);
          RightDown();
          delay(1000+i*5);//multiplier is because bigger loads take more time
          ReadInputPins();
          KillAll();
          delay(500);

          // record these values now (fixes bug related to ubidots output being overwritten by a later call to ReadInputPins)
          float tempMF = measuredPullForce;
          float tempFS = forceSetting;
          float tempPR = rightDucerPosInch;

          PrintDiagnostic("FvD "+ String(i));
          PrintStatusToLCD("FvD "+ String(i));

          // reset position by pushing up
          SetForce(5);
          PushUp();
          delay(1000+i*5);
          KillAll();
          delay(500);

          if(webUpdateFlag){
              //send update to Ubidots
              request.body = "[";
              request.body += "{ \"variable\":\""WEB_FORCE_DOWN"\", \"value\": "+String(tempMF,3)+" }";
              request.body += ", { \"variable\":\""WEB_FORCE_INPUT"\", \"value\": "+String(tempFS)+" }";
              request.body += ", { \"variable\":\""WEB_COMPRESSOR_STATE"\", \"value\": "+String(compressorOn)+" }";
              request.body += ", { \"variable\":\""WEB_FORCE_UP"\", \"value\": "+String(tempPR,3)+" }";//TBD- need to add position_right to the list
              request.body += "]";
              request.path = "/api/v1.6/collections/values/";
              http.post(request, response, headers);
          }

      }
    }
    delay(500);
    ReadInputPins();
    PrintDiagnostic("Neutral");
    PrintMsg("Elastomer FvD Test Complete.");

    // reset force setting
    forceSetting = originalForceSetting;
    SetForce(forceSetting);
    KillAll();

    // increment test number
    testNumber++;
}// GenerateElastomerFvD


//------------------------------------------------------------------------
void TestRelays(){
    digitalWrite(leftDown,HIGH);
    delay(500);
    digitalWrite(leftDown,LOW);
    delay(500);

    digitalWrite(rightDown,HIGH);
    delay(500);
    digitalWrite(rightDown,LOW);
    delay(500);

    digitalWrite(leftUp,HIGH);
    delay(500);
    digitalWrite(leftUp,LOW);
    delay(500);

    digitalWrite(rightUp,HIGH);
    delay(500);
    digitalWrite(rightUp,LOW);
    delay(500);

    KillAll();

    testRelays = false;
}// TestRelays


//------------------------------------------------------------------------
// sets pressure between 3 and 100 psi, using one of two linear interpolations based on inital calibration testing
// added interpolation between 0 and 3 just in case regulator has that range
int SetPressure(float pressure, int dacNum){
    int retVal = 0;

    if(pressure<3){
        float temp = (pressure-0)/3;
        temp = temp * (intercept3psi-0) + 0;
        retVal = (int)temp;
    }
    else if(pressure<10){
        float temp = (pressure-3)/7;
        temp = temp * (intercept10psi-intercept3psi) + intercept3psi;
        retVal = (int)temp;
    }
    else if(pressure<100){
        float temp = (pressure-10)/90;
        temp = temp * (intercept100psi-intercept10psi) + intercept10psi;
        retVal = (int)temp;
    }
    else {
        retVal = intercept100psi;
    }

    if(dacNum == 1){
        pinMode(DAC1, OUTPUT);
        analogWrite(DAC1, retVal);
        DAC1_bits = retVal;
        PSI1setting = pressure;
    }
    else if(dacNum == 2){
        pinMode(DAC2, OUTPUT);
        analogWrite(DAC2, retVal);
        DAC2_bits = retVal;
        PSI2setting = pressure;
    }
    return retVal;
}// SetPressure


//------------------------------------------------------------------------
void TestMsg(String msg){
    PrintMsg(msg);
    ClearLCD();
    Serial1.write(msg);

    return;
}// TestMsg


//------------------------------------------------------------------------
float ReadDigitalPressureSensor(int pressureSensorSelector){

    // Change mux to appropriate sensor
    if(pressureSensorSelector == 0) analogWrite(muxSelectorPin, 255);
    else if(pressureSensorSelector == 1) analogWrite(muxSelectorPin, 0);
    delay(10);// tbd delay needed to switch mux?

    uint16_t byte1 = 0; // stores 1st byte
    uint8_t byte2 = 0;// stores 2nd byte
    float sum = 0;

    byte mask = 63; // 00111111
    int countsMax = 14745; //bits
    int countsMin = 1638; //bits
    int sensorMax = 150; //psi
    int sensorMin = 0; //psi

    // record two bytes of data
    Wire.requestFrom(pressureSensorAddress,2);
    if(Wire.available())
        byte1 = Wire.read();
    if(Wire.available())
        byte2 = Wire.read();
    while(Wire.available()) Wire.read(); //clear out remaining bytes (bytes 3 & 4 are garbage)

    byte1 = byte1 & mask; // eliminate bits 7 and 8 (diagnostic)
    byte1 <<=8; // shift 8 bits over
    sum = byte1 + byte2; // add the two to get the 14 bit value

    float pressureReading;
    pressureReading = (sum-countsMin);
    pressureReading = pressureReading/(countsMax-countsMin);
    pressureReading = pressureReading*(sensorMax-sensorMin);

    return pressureReading;
}// ReadDigitalPressureSensor


//------------------------------------------------------------------------
float ReadAnalogPressureSensor(){
    int pressurePos = 0; // value used to store pressure sensor Vout+
    int pressureNeg = 0; // value used to store pressure sensor Vout-
    int pressureBits = 0; // value used to store Vout+ minus Vout-
    float pressureReading = 0;  // scaled version of pressureBits in psi

    // Read pressure pins
    pressurePos = analogRead(pressurePosPin);
    pressureNeg = analogRead(pressureNegPin);

    // Calculate pressure in PSI
    pressureBits = pressurePos - pressureNeg;
    pressureReading = (pressureBits-3) * 1.23; // minus 5 is for zero offset.  scaling factor should be 0.923 is psi/bit based on Digikey P/N 480-5548-ND, but empirically it's 1.21 psi/bit
    if(pressureReading < 0) pressureReading = 0; // system can't go vacuum

    return pressureReading;
}// ReadAnalogPressureSensor


//------------------------------------------------------------------------
float SetForce(float force){
    float p1 = force/pullArea;
    float p2 = force/pushArea;

    // check that minimum tank pressure is higher than the desired output pressure and higher than the current pressure
    if(tankMin < p1){
        errorFlag = true;
        errorMsg = "Error:Tank min pressure";
        PauseAll();
        return 0;
    }

    int retVal = 0;
    retVal += SetPressure(p1,1);
    retVal += SetPressure(p2,2);

    return retVal;
}// SetForce


//------------------------------------------------------------------------
int CheckI2C(int address){
    int error;

    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0) return 1;
    else return 0;
}// CheckI2C


//------------------------------------------------------------------------
void ResetNeutral(){
    SetForce(25);
    delay(250);
    if(testMode!=1 && testMode!=5){
        PushDown();
        delay(200);
    }
    PushUp();
    delay(500);
    if(testMode==1 || testMode==5){ //when in mode 1 or 5, want "neutral" to be a lower reference point than fully topped out.
        PushDown();
        delay(300);
    }
    KillAll();
    delay(500);
    ReadInputPins();
    lastNeutralLeft = leftDucerPosInch;
    lastNeutralRight = rightDucerPosInch;
    // return to original force setting
    SetForce(forceSetting);
}// ResetNeutral

//------------------------------------------------------------------------
void LeftDown(){
    digitalWrite(leftDown,HIGH);
    digitalWrite(rightDown,LOW);
    digitalWrite(leftUp,LOW);
    digitalWrite(rightUp,LOW);
}// LeftDown

//------------------------------------------------------------------------
void RightDown(){
    digitalWrite(leftDown,LOW);
    digitalWrite(rightDown,HIGH);
    digitalWrite(leftUp,LOW);
    digitalWrite(rightUp,LOW);
}// RightDown

//------------------------------------------------------------------------
void LeftUp(){
    digitalWrite(leftDown,LOW);
    digitalWrite(rightDown,LOW);
    digitalWrite(leftUp,HIGH);
    digitalWrite(rightUp,LOW);
}// LeftUp

//------------------------------------------------------------------------
void RightUp(){
    digitalWrite(leftDown,LOW);
    digitalWrite(rightDown,LOW);
    digitalWrite(leftUp,LOW);
    digitalWrite(rightUp,HIGH);
}// RightUp

//------------------------------------------------------------------------
void PushDown(){
    digitalWrite(leftDown,HIGH);
    digitalWrite(rightDown,HIGH);
    digitalWrite(leftUp,LOW);
    digitalWrite(rightUp,LOW);
}// PushDown

//------------------------------------------------------------------------
void PushUp(){
    digitalWrite(leftDown,LOW);
    digitalWrite(rightDown,LOW);
    digitalWrite(leftUp,HIGH);
    digitalWrite(rightUp,HIGH);
}// PushUp

//------------------------------------------------------------------------
void TwistLeft(){
    digitalWrite(leftDown,HIGH);
    digitalWrite(rightDown,LOW);
    digitalWrite(leftUp,LOW);
    digitalWrite(rightUp,HIGH);
}// TwistLeft

//------------------------------------------------------------------------
void TwistRight(){
    digitalWrite(leftDown,LOW);
    digitalWrite(rightDown,HIGH);
    digitalWrite(leftUp,HIGH);
    digitalWrite(rightUp,LOW);
}// TwistRight

//------------------------------------------------------------------------
// Turns off all relays & turns pressure to minimum
void KillAll(){
    digitalWrite(leftDown,LOW);
    digitalWrite(rightUp,LOW);
    digitalWrite(rightDown,LOW);
    digitalWrite(leftUp,LOW);
    SetForce(0);
}// KillAll

//------------------------------------------------------------------------
// Turns off all relays
void KillRelays(){
    digitalWrite(leftDown,LOW);
    digitalWrite(rightUp,LOW);
    digitalWrite(rightDown,LOW);
    digitalWrite(leftUp,LOW);
}// KillRelays

//===================================================================================================================
//Pauses cycles, turns off relays, continues to update screen, waits for un-pause
void PauseAll(){
    //turn off all relays
    KillAll();

    // wait for error flag to be reset
    while(errorFlag){
        ReadInputPins();
        PrintStatusToLCD(errorMsg);
        if(webUpdateFlag) UpdateDashboard();
        delay(200);
    }

    // wait for pause button to be toggled off
    while(paused){
        ReadInputPins();
        PrintStatusToLCD("Paused");
        if(webUpdateFlag) UpdateDashboard();
        RunCalibrations();// runs functions enabled through webhooks
        delay(200);
    }

    SetForce(forceSetting);
    ClearLCD();

    // Reset timers
    compressorStartTime = millis();
    compressorStopTime = millis();


}// PauseAll


//===================================================================================================================
// Spark.functions (registered in Setup() above)
// Spark.functions always take a string as an argument and return an integer.
int WebRunFunction(String command) {

    if(command=="leftUp") {
        testRelayPin = leftUp;
        testRelay = true;
        return 1;
    }
    else if(command=="leftDown") {
        testRelayPin = leftDown;
        testRelay = true;
        return 1;
    }
    else if(command=="rightUp") {
        testRelayPin = rightUp;
        testRelay = true;
        return 1;
    }
    else if(command=="rightDown") {
        testRelayPin = rightDown;
        testRelay = true;
        return 1;
    }
    else if(command=="calibrateUpDown") {
        calibrateInPhase = true;
        return 1;
    }
    else if(command.substring(0,14)=="calibrateTwist") {
        command = command.substring(14);
        int tempLoad = command.toInt();
        if(tempLoad==tempLoad) calibrateTwistLoad = tempLoad; // check if NaN
        else return -1;
        calibrateOutOfPhase = true;
        return calibrateTwistLoad;
    }
    else if(command=="calibratePressure"){
        calibratePressure = true;
        return 1;
    }
    else if(command=="calibrateAll"){
        calibrateOutOfPhase = true;
        calibrateInPhase = true;
        return 1;
    }
    else if(command=="calibrateWindow"){
        calibrateWindow = true;
        return 1;
    }
    else if(command.substring(0,11)=="generateFvD"){
        command = command.substring(11);
        int tempLoad = command.toInt();
        if(tempLoad==tempLoad) maxTestLoad_Bar = tempLoad; // check if NaN
        else return -1;
        generateFvD = true;
        return maxTestLoad_Bar;
    }
    else if(command.substring(0,12)=="elastomerFvD"){
        command = command.substring(12);
        if(command.substring(0,1)=="*") {
          command = command.substring(1);
          iterateFvD = true;
        }
        else iterateFvD = false;

        int tempLoad = command.toInt();
        if(tempLoad==tempLoad) maxTestLoad_Elastomer = tempLoad; // check if NaN
        else return -1;
        elastomerFvD = true;
        return maxTestLoad_Elastomer;
    }
    else if(command.substring(0,13)=="continuousFvD"){
        command = command.substring(13);
        int tempLoad = command.toInt();
        if(tempLoad==tempLoad) maxTestLoad_Elastomer = tempLoad; // check if NaN
        else return -1;
        continuousFvD = true;
        return maxTestLoad_Elastomer;
    }
    else if(command=="diagnostic") {
        PrintDiagnostic("");
        return 1;
    }
    else if(command=="pause"){
        if(paused) paused = false;
        else paused = true;
        return paused;
    }
    else if(command=="I2C"){
        if(useI2C) useI2C = false;
        else useI2C = true;
        return useI2C;
    }
    else if(command=="pressureGauge"){
        if(usePressureGauge) usePressureGauge = false;
        else usePressureGauge = true;
        return usePressureGauge;
    }
    else if(command=="errorChecking"){
        if(errorChecking) errorChecking = false;
        else errorChecking = true;
        return errorChecking;
    }
    else if(command=="web"){
        if(webUpdateFlag) webUpdateFlag = false;
        else webUpdateFlag = true;
        return webUpdateFlag;
    }
    else if(command.substring(0,7)=="webRate"){
        command = command.substring(7);
        dashboardRefreshRate = command.toInt();
        return dashboardRefreshRate;
    }
    else if(command=="testRelays") {
        testRelays = true;
        return 1;
    }
    else if(command=="neutral") {
        ResetNeutral();
        return 1;
    }
    else if(command.substring(0,4)=="mode"){
        command = command.substring(4,5);
        testMode = command.toInt();
        SetMode(testMode);
        return testMode;
    }
    else if(command.substring(0,5)=="count"){
        command = command.substring(5);
        cycleCount = command.toInt();
        return cycleCount;
    }
    else if(command.substring(0,6)=="target"){
        command = command.substring(6);
        cycleTarget = command.toInt();
        return cycleTarget;
    }
    else if(command.substring(0,4)=="tmax"){
        command = command.substring(4);
        tankMax = command.toInt();
        return tankMax;
    }
    else if(command.substring(0,4)=="tmin"){
        command = command.substring(4);
        tankMin = command.toInt();
        return tankMin;
    }
    else if(command.substring(0,4)=="test"){
        command = command.substring(4);
        testNumber = command.toInt();
        return testNumber;
    }
    else if(command.substring(0,6)=="buffer"){
        command = command.substring(6);
        measuredForceBuffer = command.toInt();
        return measuredForceBuffer;
    }
    else if(command=="resetError"){
        errorFlag = false;
        errorMsg = "";
        return 1;
    }
    else if(command=="disp"){
        if(disp1) disp1 = 0;
        else disp1 = 1;
        return 1;
    }
    else if(command=="status"){
      return cycleCount;
    }
    else if(command=="version"){
      return version;
    }
    else if(command=="tps"){
    TestPressureSensors();
    }
    else{
        TestMsg(command);
        return 2;
    }

    return 0;
}// WebRunFunction


//------------------------------------------------------------------------
int WebTestPressure(String pressureStr){
    int pressure = pressureStr.toInt();

    SetPressure(pressure,1);
    SetPressure(pressure,2);

    return pressure;
}// WebTestPressure


//------------------------------------------------------------------------
int WebSetForce(String forceStr){
    forceSetting = forceStr.toFloat();
    SetForce(forceSetting);
//    calibrateWindow = true; // re-calibrate after changing force
    return forceSetting;
}// WebSetForce


//------------------------------------------------------------------------
int WebSetTimeout(String tStr){

    // if 1st char is a "*", only update state 1 (up direction)
    if(tStr.substring(0,1)=="*"){
        stateTimeout[1] = tStr.substring(1).toInt();
        return stateTimeout[1];
    }
    else{
        stateTimeout[0] = tStr.toInt();
        stateTimeout[1] = tStr.toInt();
        stateTimeout[2] = tStr.toInt();
        return stateTimeout[0];
    }
}// WebSetTimeout


void TestPressureSensors(){

  SetPressure(10,1);
  SetPressure(10,2);
  PushDown();
  delay(1000);
  ReadDigitalPressureSensor(0);
  delay(500);
  WriteLineToLCD("0: ps " + String(pushPressurePSI,1) + " pl " + String(pushPressurePSI,1),1);
  delay(500);
  ReadDigitalPressureSensor(1);
  delay(500);
  WriteLineToLCD("1: ps " + String(pushPressurePSI,1) + " pl " + String(pushPressurePSI,1),2);
  delay(2000);
  KillRelays();

  SetPressure(12,1);
  SetPressure(12,2);
  PushUp();
  delay(1000);
  ReadDigitalPressureSensor(0);
  delay(500);
  WriteLineToLCD("0: ps " + String(pushPressurePSI,1) + " pl " + String(pullPressurePSI,1),1);
  delay(500);
  ReadDigitalPressureSensor(1);
  delay(500);
  WriteLineToLCD("1: ps " + String(pushPressurePSI,1) + " pl " + String(pullPressurePSI,1),2);
  delay(2000);
  KillRelays();

}
