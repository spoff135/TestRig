// Regulator 1/DAC 1 is pull (down), Pressure Sensor 1
// Regulator 2/DAC 2 is push (up), Pressure Sensor 0

String version = "1.22.1a"; // version number (month.day.rev)
int testMode = 0; // current mode (0=no test running,1=in-phase test,2=out-of-phase test,3=realworld in-phase, 5=elastomer testing right only, 6=elastomer testing both cylinders, 7=ShockStop Up-only ISO, 8=Seatpost ISO Test (right only), 9=Aerobars Extension Up/Down)
int cycleCount = 0;
int cycleTarget = 100000;
int stateTimeout[3] = {2000,2000,2000}; // time (ms) before automatic state change
float neutralPosThreshold = 1.000; // (in) thershold whereby the software determines that the seatpost has not fully returned to the neutral position (i.e. plastic deformation has occured)
float windowExcursionLimit = 1.15; // % excursion allowed from defined calibration window (1.15 = 15% window)
bool webUpdateFlag = true;
bool errorChecking = true;
int webDashboardRefreshRate = 5000; // web dashboard refresh rate (ms)
int webUpdateConstantsRate = 10000; // web constants refresh rate (ms)
int testNumber = 1; // automatically incremented with each test
float mode7Ratio = 0.2; // ratio of downforce to upforce in up-only (mode 7) test


// Initialize the LCD library
#include "Serial_LCD_SparkFun.h"
Serial_LCD_SparkFun lcd = Serial_LCD_SparkFun();
// LCD Initialization


//UBIDOTS CODE
#include "HttpClient.h"  // if using webIDE, use this: #include "HttpClient/HttpClient.h"

/*
//Test Rig One Variable IDs
#define WEB_DEFLECTION "56cc63b6762542644cc8d2ef"
#define WEB_DEFLECTION_AVG "56b671fb76254228866ee416"
#define WEB_DEFLECTION_LIMIT "57acfcf47625425735b8b407"
#define WEB_CYCLES "56b672cb7625422dd8dbbf52"
#define WEB_FORCE_S1 "56cc62a27625425dba666c80"
#define WEB_FORCE_S2 "56cc666d7625427368e80d6e"
#define WEB_FORCE_INPUT "56cc62db7625425f3cd6ae79"
#define WEB_POSITION_S1_RIGHT "57923c897625423072003e71"
#define WEB_POSITION_S2_RIGHT "57923c937625423072003ec0"
#define WEB_POSITION_S1_MIN "57923cfe76254237a3daea81"
#define WEB_POSITION_S2_MIN "57923d0476254237cebbf5c3"
#define WEB_TEST_STATUS "56cc62f37625425f3cd6aeb2"
#define WEB_PUSH_FORCE_SETTING "59775e05c03f9769a7500695"
#define WEB_PULL_FORCE_SETTING "59775e0cc03f9769d9387961"
#define WEB_S1_TIMEOUT "5977638cc03f976e9d662cb7"
#define WEB_S2_TIMEOUT "59776393c03f976efe84e3fd"
#define WEB_PERIOD "5979db17c03f973987af9917"
#define WEB_CYCLE_TARGET "5979db98c03f973acdd79192"
*/

//Test Rig Two Variable IDs
#define WEB_DEFLECTION "5a6608c6c03f970476ab839d"
#define WEB_DEFLECTION_AVG "5a6608e3c03f970478f74d9e"
#define WEB_DEFLECTION_LIMIT "5a66089dc03f970478f74d92"
#define WEB_CYCLES "5a6608dbc03f9704e880034e"
#define WEB_FORCE_S1 "5a6608bcc03f970474113e51"
#define WEB_FORCE_S2 "5a6608c2c03f970478f74d99"
#define WEB_FORCE_INPUT "5a6608d3c03f97056c78672a"
#define WEB_POSITION_S1_RIGHT "5a6608b0c03f970478f74d96"
#define WEB_POSITION_S2_RIGHT "5a6608b5c03f970478f74d98"
#define WEB_POSITION_S1_MIN "5a6608a4c03f97047b758c6c"
#define WEB_POSITION_S2_MIN "5a6608aac03f9704e8800346"
#define WEB_TEST_STATUS "5a6608ccc03f970476ab839f"
#define WEB_PUSH_FORCE_SETTING "5a660893c03f970478f74d91"
#define WEB_PULL_FORCE_SETTING "5a66088bc03f970478f74d90"
#define WEB_S1_TIMEOUT "5a6607d7c03f970476ab8383"
#define WEB_S2_TIMEOUT "5a6607cec03f9703c6557126"
#define WEB_PERIOD "5a6607b8c03f97032077c07f"
#define WEB_CYCLE_TARGET "5a53a746c03f9737a2820c11"

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


// Strain Gauge Setup Code
#include "HX711.h"
#define DOUT  6
#define CLK  7
HX711 scale(DOUT, CLK);
float SG_calibrationFactor = -9920; //Calibrated to -9920 on 9/21/17 with a range of weights up to 300lbs

float SG_zeroFactor = 0;
float SG_tareFactor = 0;

float SG_measuredForceAbsolute = 0;
float SG_AvgSampleRate = 0;
unsigned int SG_samplesTakenThisState = 0;
unsigned long SG_previousMillis = 0;
unsigned long SG_currentMillis = 0;
// Strain Gauge

int LCDrefreshRate = 300; //LCD refresh rate (ms)

// Define digital pins
int leftUp = D2; // relay 1
int rightUp = D3; // relay 2
int leftDown = D4; // relay 3
int rightDown = D5; // relay 4
// PIN D6 is used for strain gauge
// PIN D7 is used for strain gauge
int testRelayPin = 0; // gets reassigned in code and used by calibrate function to test individual relays

// Define analog pins
// tbd deleteme int muxSelectorPin = WKP; // analog pin (used as digital) associated with
int leftDucerPin = A2; //analog pin associated with the left transducer
int rightDucerPin = A1; //analog pin associated with the right transducer

// Define system hardware constants

// ShockStop L/R Cylinders
//float pullArea = 2.8348; // mcm part 6498K477
//float pushArea = 3.1416; // mcm part 6498K477
//float ducerInchesPerBit = 0.00096118; // based on 100mm ducer/4096 bits = .00096118"/bit

// Seatpost Cylinder & ducer
float pullArea = 4.6019; // mcm part 6498K488 / 6498K491
float pushArea = 4.9087; // mcm part 6498K488 / 6498K491
float ducerInchesPerBit = 0.00144177; // based on 150mm ducer/4096 bits = .00096118"/bit

// Aerobars Extension Cylinder
//float pullArea = 0.552; // bimba
//float pushArea = 0.601; // bimba
//float ducerInchesPerBit = tbd? 0.00144177 // based on 150mm ducer/4096 bits = .00096118"/bit

int minPSI = 3; // regulator min pressure
int maxPSI = 120; // regulator max pressure
int maxResolution = 4095;
int maxTestLoad_Elastomer = 200; // maximum test force used in GenerateElastomerFvD
int maxTestLoad_Seatpost = 200; // maximum test force used in GenerateSeatpostFvD
int maxTestLoad_Bar = 120; // maximum force (per cylinder) used in GenerateFvD
int calibrateTwistLoad = 65; // maximum test force used in calibrateTwist
    // Define I2C addresses
int dataLoggerAddress = 8; // address of dataLogger
    // Regulator calibration settings
int intercept100psi = 3550;
int intercept10psi = 515;
int intercept3psi = 250;
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
    // timing variables
long timeLeft = 0; // estimated time remaining to reach cycleTarget
long lastLCDupdate = 0; //time of last LCD update
long lastI2Cupdate = 0; // time of last I2C status check
long lastDashboardUpdate = 0; // time of last dashboard update
long lastWebConstantsUpdate = 0; // time of last web constants update
long stateStartTime = 0; // start time (ms) of current state
long stateTime = 0; // elapsed time (ms) in current state
long period = 0; // total time to go through an entire cycle
    // outputs state flags/variables
bool paused = true;
bool statusUpdate = true;
bool webUpdateConstants = true; // used to send data web dashboard that doesn't change often
bool useI2C = true;
bool useZeroPSI = false;
bool calibratePressure = false;
int currentState = 0;  // current state (0=null,1 & 2 are mode-dependent)
int DAC1_bits = 0; // current setting of DAC1
int DAC2_bits = 0; // current setting of DAC2
float PSI1setting = 0; // current setting of DAC1 in PSI
float PSI1settingOffset = 0; // current calibration adjustment of DAC1 in PSI
float PSI2setting = 0; // current setting of DAC2 in PSI
float PSI2settingOffset = 0; // current calibration adjustment of DAC2 in PSI
float forceSetting = 0; // current force setting
String errorMsg = "None";
String statusMsg = "Boot up";
    // Measured state flags/variables
bool dataLoggerStatus = false;
bool errorFlag = false;
int displayMode = 0; // can be mode 0, 1, 2, or 3
int disp1 = 0; // display mode modifier
int leftDucerPosBits = 0; //value used to store voltage in bits
int rightDucerPosBits = 0; //value used to store voltage in bits
    // Calculated state values
float state1Force = 0; // updated in CheckStateConditions
float state2Force = 0; // updated in CheckStateConditions
float leftDucerPosInch = 0; // value used to store position in inches from zero position
float rightDucerPosInch = 0; // value used to store position in inches from zero position
    // Measured/calculated values related to position
float positionLeft[3] = {0,0,0}; // position at end of state [i]
float positionRight[3] = {0,0,0};
float refPositionLeft[3] = {0,0,0}; // reference position from calibration of state [i]
float refPositionRight[3] = {0,0,0};
float refPositionLeftWindow[3] = {0,0,0}; // max deviation allowed from reference position of state [i]
float refPositionRightWindow[3] = {0,0,0};
float positionAvg[3] = {0,0,0}; // avg position at end of state [i]
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
bool twistFvD = false;
bool calibrateWindow = false;
bool generateFvD = false; // generates force vs displacement graph
bool elastomerFvD = false; // generates force vs displacement graph for elastomers
bool seatpostFvD = false; // generates force vs displacement graph for seatposts
bool contSeatpostFvD = false; // generates continuous force vs displacement graph for seatposts
bool iterateFvD = false; // flag for iterative FvD graph
bool continuousFvD = false; // flag for continuous force vs displacement graph for elastomers

void setup()
{
    Wire.begin(); // join i2c bus (address optional for master)
    Serial1.begin(9600); // Initialize serial line for LCD
    InitializeLCD(); // Initialize LCD
    lcd.setCursor(1,1);
    Serial1.print("LCD Initialized.");

    // Configure digital pins as input or output
    pinMode(leftUp, OUTPUT);
    pinMode(rightUp, OUTPUT);
    pinMode(leftDown, OUTPUT);
    pinMode(rightDown, OUTPUT);

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

    // Tare Strain Gauge
    lcd.setCursor(1,1);
    Serial1.print("Tare-ing Scale...");
    delay(1000);
    scale.set_scale(SG_calibrationFactor); //Adjust to this calibration factor
    scale.tare(); //Reset the scale to 0
    lcd.clear();

    //UBIDOTS CODE
    request.hostname = "things.ubidots.com";
    request.port = 80;

    lcd.clear();
    lcd.setCursor(1,1);
    Serial1.print("Starting Program");
}// Setup


void loop()
{
    ReadInputPins();
    RunCalibrations();// runs functions enabled through webhooks

    if(webUpdateFlag) UpdateDashboard();

    bool stateChange = false;
    currentState = 1; // start in state 1.
    SetState(currentState);
    stateStartTime = millis();

    if(cycleCount>=cycleTarget) paused=true; // pause test if cycleTarget is reached

    while(!paused && currentState<=nStates && testMode>0){
      ReadInputPins();
      stateTime = millis()-stateStartTime;
      stateChange = CheckStateConditions(currentState);
      if(stateChange){
        SG_samplesTakenThisState = 0; // reset counter

        // Before changing state, update positions
        positionLeft[currentState] = leftDucerPosInch;
        positionRight[currentState] = rightDucerPosInch;
        positionAvg[currentState] = (rightDucerPosInch + leftDucerPosInch)/2;

        if(!paused){
          currentState = currentState+1;
          SetState(currentState);
          stateStartTime = millis();
        }
        else break;
      }
      PrintStatusToLCD("Run");
    }

    // Update deflection total and average
    deflection = CalculateDeflection();
    deflectionAvg = 0.9*deflectionAvg + 0.1*deflection;

    CheckForErrors();

    // Only count one error per cycle
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
        errorCountConsecutive = 0;
        statusMsg = "Error";
    }

    if(paused){
        PauseAll();
    }

}// loop


//------------------------------------------------------------------------
void CheckForErrors(){

  if(testMode>0 && testMode!=9){ // no position error checking for tests without linear transducers

    // check positions/deflections against against window
    if(deflection > deflectionMax){
      errorFlag = true;
      errorMsg = "Total defl error";
    }
    for(int i=1; i<=nStates; i++){
      if(testMode!=5 && testMode!=8 && testMode!=9){ //Don't check left side for 1-cylinder test modes TBD Update w/other modes
        if(abs(positionLeft[i]-refPositionLeft[i]) > refPositionLeftWindow[i]){
          errorFlag = true;
          errorMsg = "Error: LeftPos, S" + String(i);
        }
      }
      if(abs(positionRight[i]-refPositionRight[i]) > refPositionRightWindow[i]){
        errorFlag = true;
        errorMsg = "Error: RightPos, S" + String(i);
      }
    }
  }
}
// CheckForErrors

//------------------------------------------------------------------------
bool CheckStateConditions(int currentState){
  bool stateChange = false;

  switch (currentState) {
  case 0:
    stateChange = true;
    break;
  case 1:
    if(stateTime > stateTimeout[currentState]) stateChange = true;
    if(stateChange){
      state1Force = SG_measuredForceAbsolute; // Record load at end of state

      // For most ISO test modes, adjust PUSH pressure settings
      if(testMode==1 || testMode==2 || testMode==7 || testMode==9){
        if(state1Force < forceSetting){
          PSI2settingOffset+=0.1;
          SetPressure(PSI2setting,2);
        }
        else if(state1Force > forceSetting){
          PSI2settingOffset-=0.1;
          SetPressure(PSI2setting,2);
        }
      }
      // For Seatpost ISO test mode, adjust PULL pressure settings
      else if(testMode==8){
        if(state1Force < forceSetting){
          PSI1settingOffset+=0.1;
          SetPressure(PSI1setting,1);
        }
        else if(state1Force > forceSetting){
          PSI1settingOffset-=0.1;
          SetPressure(PSI1setting,1);
        }
      }
    }
    break;
  case 2:
    if(stateTime > stateTimeout[currentState]) stateChange = true;
    if(stateChange){
      state2Force = SG_measuredForceAbsolute; // Record load at end of state

      // For most ISO test modes, adjust PULL pressure settings
      if(testMode==1 || testMode==2 || testMode==3 || testMode==7 || testMode==9){
        if(testMode==7) state2Force = state2Force/mode7Ratio; // for up-only test, scale up reading to match up force
        if(state2Force < forceSetting){
          PSI1settingOffset+=0.1;
          SetPressure(PSI1setting,1);
        }
        else if(state2Force > forceSetting){
          PSI1settingOffset-=0.1;
          SetPressure(PSI1setting,1);
        }
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
    else if(testMode==6){ // elastomer testing (both cylinders)
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
    else if(testMode==7){ // up-only ISO in-phase testing
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
    else if(testMode==8){ // Seatpost ISO test (use right cylinder in pull mode)
      switch (currentState) {
      case 1:
        RightDown();
        break;
      case 2:
        KillRelays();
        cycleCount++;
        break;
      }
    }
    else if(testMode==9){ // Aerobars Extension Up/Down (use right cylinder)
      switch (currentState) {
      case 1:
        RightUp();
        break;
      case 2:
        RightDown();
        cycleCount++;
        break;
      }
    }
    else {
      KillRelays();
    }
}// SetState


//------------------------------------------------------------------------
void SetMode(int testMode){

    //tbd deleteme - reset PSI setting Offsets (find a more correct place to do this?)
    PSI1settingOffset = 0;
    PSI2settingOffset = 0;

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
    case 6: // elastomer testing (both cylinders)
      testMode = 6;
      nStates = 2;
      break;
    case 7: // up-only ISO in-phase testing
      testMode = 7;
      nStates = 2;
      break;
    case 8: // Seatpost ISO test
      testMode = 8;
      nStates = 2;
      break;
    case 9: // Aerobars Extension Up/Down
      testMode = 9;
      nStates = 2;
      break;
    default:
      testMode = 0;
      nStates = 0;
      KillAll();
      break;
    }

}// SetMode


//------------------------------------------------------------------------
float CalculateDeflection(){

    float leftDef = positionLeft[2]-positionLeft[1];
    if(leftDef < 0) leftDef = leftDef*-1;

    float rightDef = positionRight[2]-positionRight[1];
    if(rightDef < 0) rightDef = rightDef*-1;

    float totalDef = 0;
    if(testMode==5 || testMode==8 || testMode==9){// for seatpost ISO test, just return right deflection
       totalDef = rightDef;
    }
    else{
      totalDef = (leftDef + rightDef)/2;
    }
    return totalDef;
}// CalculateDeflection


//------------------------------------------------------------------------
void InitializeLCD(){
  // Setup LCD
  lcd.setBrightness(20);
  lcd.clear();
}// InitializeLCD


//------------------------------------------------------------------------
// Reads input pins and updates the appropriate variables
void ReadInputPins(){

    // Check to see if I2C hardware is responding
    if(useI2C && millis()-lastI2Cupdate > LCDrefreshRate){
      dataLoggerStatus = CheckI2C(dataLoggerAddress);
      lastI2Cupdate = millis();
    }

    // Read Strain Gauge
    if(scale.is_ready()){
      SG_measuredForceAbsolute = abs(scale.get_units(1));
      SG_currentMillis = millis();
      SG_AvgSampleRate = SG_AvgSampleRate*0.9 + (1000/ (SG_currentMillis - SG_previousMillis))*0.1;
      SG_previousMillis = SG_currentMillis;
      SG_samplesTakenThisState++;
    }

    // Read transducer (position) pins
    leftDucerPosBits = analogRead(leftDucerPin);
    rightDucerPosBits = analogRead(rightDucerPin);
    leftDucerPosInch = leftDucerPosBits * ducerInchesPerBit;
    rightDucerPosInch = rightDucerPosBits * ducerInchesPerBit;

    // update timeLeft
    timeLeft =  ( (cycleTarget-cycleCount) * period) / 60000.0;
}// ReadInputPins


//------------------------------------------------------------------------
void RunCalibrations(){
  if(calibrateInPhase){
    CycleInPhase(5,30);
    calibrateInPhase = false;
  }

  if(calibratePressure){
    CalibratePressure();
    calibratePressure = false;
  }

  if(generateFvD){
    GenerateFvD();
    generateFvD = false;
  }

  if(twistFvD){
    GenerateTwistFvD();
    twistFvD = false;
  }

  if(elastomerFvD){
    GenerateElastomerFvD();
    elastomerFvD = false;
  }

  if(seatpostFvD){
    GenerateSeatpostFvD();
    seatpostFvD = false;
  }

  if(contSeatpostFvD){
    GenerateContinuousSeatpostFvD();
    contSeatpostFvD = false;
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
    ReadInputPins();
    delay(150);
    PrintDiagnostic("testRelay");
    digitalWrite(testRelayPin,LOW);
    testRelay = false;
  }

  bool stateChange = false;
  if(calibrateWindow){
    if(testMode!=4){
      ResetNeutral();
    }

    if(testMode==8){//tbd set this up for all modes, not just ISO seatpost
      PSI1settingOffset = FindPSISettingOffset(forceSetting,1,1);
    }
    SetForce(forceSetting); //Adjust force after determining offset

    // run through each state and record the final deflection to set deflectionWindow
    int i;
    int j;
    int numCalibrationCycles = 5;
    for(j=0;j<numCalibrationCycles;j++){
      for(i=1;i<=nStates;i++){
        PrintStatusToLCD("Calibrate State " + String(i));
        SetState(i);
        stateStartTime = millis();
        stateChange = false;

        while(stateChange == false){
          ReadInputPins();
          stateTime = millis()-stateStartTime;
          stateChange = CheckStateConditions(i);
        }

        refPositionLeft[i] = leftDucerPosInch;
        refPositionRight[i] = rightDucerPosInch;

        positionLeft[i] = leftDucerPosInch; // added these so "CalculateDeflection" has something to reference.
        positionRight[i] = rightDucerPosInch;
      }
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
    if(testMode!=4){
      ResetNeutral();
    }
    delay(2000);
  }
}// RunCalibrations


//------------------------------------------------------------------------
// Prints measured and state variables to LCD
void PrintStatusToLCD(String origMsg){
    String msg = "";
    if(millis()-lastLCDupdate > LCDrefreshRate){
        if(displayMode==0){
          lcd.clear();

//          msg = "SG " + String(SG_measuredForceAbsolute,2) + "|Pr " + String(measuredPullForcePressureSensor,2) + "         ";
          lcd.setCursor(1,1);
          Serial1.print(msg);
          lcd.setCursor(2,1);
//          Serial1.print(String(pullPressurePSI,2) + "PSI    ");

//          WriteLineToLCD(msg,2);
/*            ClearLCD();

            msg = "Ver:" + version + " E"+ String(errorCountConsecutive) + " M" + String(testMode);
            msg += "                "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
            WriteLineToLCD(msg,1);

            msg = "#" + String(cycleCount) + "/" + String(cycleTarget);
            msg += " " + String(timeLeft) + "min";
            msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
            WriteLineToLCD(msg,2);

            if(useI2C) msg = origMsg + " |D" + String(dataLoggerStatus) + " P" + String(pressureSensorStatus) + " S" + String(currentState) + " T" + String(testNumber);
            else msg = origMsg + " | I2C Off";
            msg += "         ";
            WriteLineToLCD(msg,3);

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
//USE THIS FOR DIAGNOSING PRESSURE GAUGE READINGS
        msg = origMsg;
        msg += "Ver:" + version;
        msg += " Fs:" + String(forceSetting,0);
        msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
        WriteLineToLCD(msg,1);

        msg = "Ps" + String(PSI1setting,1);
        msg += " P" + String(pullPressurePSI,1);
        msg += " Pll:" + String(measuredPullForcePressureSensor,1);
        msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
        WriteLineToLCD(msg,2);

        msg = "Ps" + String(PSI2setting,1);
        msg += " P" + String(pushPressurePSI,1);
        msg += " Psh:" + String(measuredPushForcePressureSensor,1);
        msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
        WriteLineToLCD(msg,3);

        msg = "Defl " + String(deflection,2);
        msg += "/" + String(deflectionAvg,3);
        msg += "/" + String(deflectionMax,3);
        msg += "         ";
        WriteLineToLCD(msg,4);
      }
      else if(displayMode==3){
// USE THIS FOR VIEWING CYLINDER POSITIONS LIVE
        msg = origMsg;
        msg += "                 "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
        WriteLineToLCD(msg,1);

        msg = "L: " + String(leftDucerPosInch,3);
        msg += " R: " + String(rightDucerPosInch, 3);
        msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
        WriteLineToLCD(msg,2);

        msg = "                  "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
        WriteLineToLCD(msg,3);

        msg += "Ver:" + version;
        msg += " Fs:" + String(forceSetting,0);
        msg += "         "; // spaces added at the end of each line (so I don't have to run ClearLCD and make the screen flash)
        WriteLineToLCD(msg,4);
*/
        lastLCDupdate = millis();
      }

    }
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

  if(millis()-lastDashboardUpdate > webDashboardRefreshRate){
    //build common messages
    String StatusPostString = "";
    StatusPostString = "{\"variable\":\""WEB_TEST_STATUS"\", \"value\": "+String(!paused);
      StatusPostString += ", \"context\":{";
        StatusPostString += "\"status\": \""+statusMsg+"\", ";
        StatusPostString += "\"mode\": \""+String(testMode)+"\", ";
        StatusPostString += "\"errormsg\": \""+errorMsg+"\", ";
        StatusPostString += "\"version\": \""+version+"\"";
        StatusPostString += " }";
      StatusPostString += "}";
    String CyclesPostString = "";
    CyclesPostString = "{\"variable\":\""WEB_CYCLES"\", \"value\": "+String(cycleCount);
      CyclesPostString += ", \"context\":{";
        CyclesPostString += "\"target\": \""+String(cycleTarget)+"\", ";
        CyclesPostString += "\"ecount\": \""+String(errorLog)+"\", ";
        CyclesPostString += "\"timeout\": \""+String(stateTimeout[0])+"\", ";
        CyclesPostString += "\"period\": \""+String(period)+"\", ";
        CyclesPostString += "\"window\": \""+String((windowExcursionLimit-1)*100,0)+"\"";
        CyclesPostString += " }";
      CyclesPostString += "}";

    request.body = "[";
    request.body += StatusPostString;
    request.body += ", "+CyclesPostString;
    request.body += ", { \"variable\":\""WEB_CYCLE_TARGET"\", \"value\": "+String(cycleTarget)+" }";
    request.body += ", { \"variable\":\""WEB_DEFLECTION"\", \"value\": "+String(deflection,3)+" }";
    request.body += ", { \"variable\":\""WEB_DEFLECTION_AVG"\", \"value\": "+String(deflectionAvg,3)+" }";
    if(testMode==0){
      request.body += ", { \"variable\":\""WEB_FORCE_S1"\", \"value\": "+String(SG_measuredForceAbsolute,1)+" }";
      request.body += ", { \"variable\":\""WEB_FORCE_S2"\", \"value\": "+String(SG_measuredForceAbsolute,1)+" }";
    }
    else{
      request.body += ", { \"variable\":\""WEB_FORCE_S1"\", \"value\": "+String(state1Force,1)+" }";
      request.body += ", { \"variable\":\""WEB_FORCE_S2"\", \"value\": "+String(state2Force,1)+" }";
    }
    request.body += ", { \"variable\":\""WEB_PULL_FORCE_SETTING"\", \"value\": "+String( ((PSI1setting+PSI1settingOffset) * pullArea) ,3)+" }";
    request.body += ", { \"variable\":\""WEB_PUSH_FORCE_SETTING"\", \"value\": "+String( ((PSI2setting+PSI2settingOffset) * pushArea) ,3)+" }";
    request.body += ", { \"variable\":\""WEB_S1_TIMEOUT"\", \"value\": "+String(stateTimeout[1])+" }";
    request.body += ", { \"variable\":\""WEB_S2_TIMEOUT"\", \"value\": "+String(stateTimeout[2])+" }";
    request.body += ", { \"variable\":\""WEB_PERIOD"\", \"value\": "+String(period)+" }";
    request.body += ", { \"variable\":\""WEB_POSITION_S1_RIGHT"\", \"value\": "+String(positionRight[1],3)+" }";
    request.body += ", { \"variable\":\""WEB_POSITION_S2_RIGHT"\", \"value\": "+String(positionRight[2],3)+" }";
    request.body += ", { \"variable\":\""WEB_POSITION_S1_MIN"\", \"value\": "+String(refPositionRight[1]-refPositionRightWindow[1],3)+" }";
    request.body += ", { \"variable\":\""WEB_POSITION_S2_MIN"\", \"value\": "+String(refPositionRight[2]-refPositionRightWindow[2],3)+" }";
    if(webUpdateConstants || millis()-lastWebConstantsUpdate > webUpdateConstantsRate) {
      request.body += ", { \"variable\":\""WEB_FORCE_INPUT"\", \"value\": "+String(forceSetting,1)+" }";
      request.body += ", { \"variable\":\""WEB_DEFLECTION_LIMIT"\", \"value\": "+String(deflectionMax,3)+" }";
      webUpdateConstants = false;
    }
    request.body += "]";
    request.path = "/api/v1.6/collections/values/";
    http.post(request, response, headers);

    lastDashboardUpdate = millis();
  }
}// UpdateDashboard


//------------------------------------------------------------------------
void PrintDiagnostic(String stateStr){
  // Message Header "Time,SystemState,cycleCount,testNumber,ForceSet,PSI1set,PSI2set,pullPressurePSI,pushPressurePSI,DAC1_bits,leftDucerPosBits,rightDucerPosBits,leftDucerPosInch,rightDucerPosInch,lastNeutralLeft,lastNeutralRight,SG_measuredForceAbsolute"
  String msg = "";

  msg += String(Time.now()) + ",";
  msg += stateStr + ",";
//    msg += String(measuredPushForcePressureSensor,2) + ",";
//    msg += String(measuredPullForcePressureSensor,2) + ",";
  msg += String(cycleCount) + ",";
  msg += String(testNumber) + ",";
  msg += String(forceSetting,2) + ",";
  msg += String(PSI1setting,2) + ","; //pull
  msg += String(PSI2setting,2) + ","; //push
  msg += "pullPSI";//String(pullPressurePSI,2) + ",";
  msg += "pushPSI";//String(pushPressurePSI,2) + ",";
  msg += String(DAC1_bits) + ",";
  msg += String(leftDucerPosBits) + ",";
  msg += String(rightDucerPosBits) + ",";
  msg += String(leftDucerPosInch,3) + ",";
  msg += String(rightDucerPosInch,3) + ",";
  msg += String(lastNeutralLeft,3) + ",";
  msg += String(lastNeutralRight,3) + ",";
  msg += String(SG_measuredForceAbsolute,3);

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
  int delayMS = 500;

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

  KillRelays();

  PrintMsg("Test Complete.");
}// CycleOutOfPhase


//------------------------------------------------------------------------
void CalibratePressure(){
    PrintMsg("Calibrating Pressure");

    int j =0;

    for(i=minPSI;i<100;i++){
        SetPressure(i,1);
        SetPressure(i,2);
        delay(750);
        ReadInputPins();
        delay(250);
        PrintDiagnostic("");
        PrintStatusToLCD("");
    }

    for(i=minPSI;i<100;i+=10){
        SetPressure(i,1);
        SetPressure(i,2);
        delay(2750);
        ReadInputPins();
        delay(250);
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


//------------------------------------------------------------------------
void GenerateTwistFvD(){
  int originalForceSetting = forceSetting; // record original force setting to reset to after test

  for(i=5; i<=calibrateTwistLoad; i+=5)
  CycleOutOfPhase(1,i);

  // reset force setting
  forceSetting = originalForceSetting;
  SetForce(forceSetting);

  // increment test number
  testNumber++;
}// GenerateTwistFvD


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
}// GenerateContinuousFvD


//------------------------------------------------------------------------
void GenerateElastomerFvD(){
    int originalForceSetting = forceSetting; // record original force setting to reset to after test

    SetForce(5);
    delay(250);
    for(i=0; i<3; i++){ // cycle max load down 3 times to "set" elastomer
      SetForce(maxTestLoad_Elastomer);
      if(testMode==5) RightDown();
      else if(testMode==6) PushDown();
      delay(1000);
      PrintStatusToLCD("");
      SetForce(5);
      if(testMode==5) RightUp();
      else if(testMode==6) PushUp();
      delay(2000);
      PrintStatusToLCD("");
    }
    KillAll();
    delay(10000); // delay 10s to allow elastomer to recover

    // Record max up position
    SetForce(5);
    delay(250);
    if(testMode==5) RightUp();
    else if(testMode==6) PushUp();
    delay(1000);
    ReadInputPins();
    PrintDiagnostic("Up");
    PrintStatusToLCD("Up");

    // Record "neutral" position
    SetForce(0);
    if(testMode==5) RightDown();
    else if(testMode==6) PushDown();
    delay(1000);
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

          if(testMode==5) RightDown();
          else if(testMode==6) PushDown();

          delay(1000+i*5);//multiplier is because bigger loads take more time
          ReadInputPins();
          // record these values now (fixes bug related to ubidots output being overwritten by a later call to ReadInputPins)
          float tempMF = SG_measuredForceAbsolute;
          float tempFS = forceSetting;
          float tempPR = rightDucerPosInch;

          PrintDiagnostic("FvD "+ String(i));
          PrintStatusToLCD("FvD "+ String(i));

          KillAll();
          delay(500);

          // reset position by pushing up
          SetForce(5);
          if(testMode==5) RightUp();
          else if(testMode==6) PushUp();
          delay(1000+i*5);

          KillAll();
          delay(500);
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
void GenerateSeatpostFvD(){
    int originalForceSetting = forceSetting; // record original force setting to reset to after test

    // Record max up position
    SetForce(5);
    delay(250);
    RightUp();
    delay(1000);
    ReadInputPins();
    PrintSeatpostFvD("Up");

    // Record "neutral" position
    SetForce(0);
    RightDown();
    delay(1000);
    KillAll();
    delay(1000);
    ReadInputPins();
    delay(200);
    PrintSeatpostFvD("Neutral");
    float neutralPosInch = rightDucerPosInch;
    float maxStart;
    float maxLoad;
    if(iterateFvD) maxStart = 0;
    else maxStart = maxTestLoad_Seatpost;

    RightDown();

    // outer loop allows for iterativeFvD
    for(maxLoad = maxStart; maxLoad <= maxTestLoad_Seatpost; maxLoad+=10){
      // inner loop records force vs displacement from 0 to maxLoad
      for(i=0; i<=maxLoad; i+=0.5){
/*tbd deleteme
          // increment by 3 lbs in the beggining
          if(i>16) i+=7; // after 16 lbs, increment by a total of 10 lbs (3+7)
          delay(600+i*5);//multiplier is because bigger loads take more time
          SetForce(i);
          delay(400);

*/
        forceSetting = i;
        SetForce(i);
        delay(250);
        ReadInputPins();
        // record these values now (fixes bug related to ubidots output being overwritten by a later call to ReadInputPins)
//tbd deleteme        float tempMF = measuredPullForcePressureSensor;
        float tempMF_SG = SG_measuredForceAbsolute;
        float tempFS = forceSetting;
        float tempPR = rightDucerPosInch;
//        lcd.clear();
//        PrintStatusToLCD("");
        PrintSeatpostFvD("FvD "+ String(i));

        /*
        RightDown();
        delay(1000+i*5);//multiplier is because bigger loads take more time
        ReadInputPins();
        // record these values now (fixes bug related to ubidots output being overwritten by a later call to ReadInputPins)
        float tempMF = measuredPullForcePressureSensor;
        float tempFS = forceSetting;
        float tempPR = rightDucerPosInch;

        PrintDiagnostic("FvD "+ String(i));
        PrintStatusToLCD("FvD "+ String(i));
        */
        delay(1000);
        KillAll();
        delay(1000+i*8);

        // Check that seatpost has returned all the way to the neutral position
        ReadInputPins();
        if(abs(rightDucerPosInch-neutralPosInch) >= neutralPosThreshold){ // error out if neutral position has shifted more than the threshold
          maxLoad = maxTestLoad_Seatpost; //break out of outer loop as well
          errorMsg = "Neutral Position Error";
          errorFlag = true;
          break;
        }
      }
    }
    delay(1000);
    ReadInputPins();
    PrintSeatpostFvD("Neutral");
    PrintMsg("Seatpost FvD Test Complete.");

    // reset force setting
    forceSetting = originalForceSetting;
    SetForce(forceSetting);
    KillAll();

    // increment test number
    testNumber++;
}// GenerateSeatpostFvD


void GenerateContinuousSeatpostFvD(){
  // Record max up position
  SetForce(5);
  delay(250);
  RightUp();
  delay(1000);
  ReadInputPins();
  PrintSeatpostFvD("Up");

  // Record "neutral" position
  SetForce(0);
  RightDown();
  delay(1000);
  KillAll();
  delay(1000);
  ReadInputPins();
  delay(200);
  PrintSeatpostFvD("Neutral");
  float neutralPosInch = rightDucerPosInch;

  float i = 0;
  float maxStart;
  float maxLoad;
  if(iterateFvD) maxStart = 0;
  else maxStart = maxTestLoad_Seatpost;

  // outer loop allows for iterativeFvD
  for(maxLoad = maxStart; maxLoad <= maxTestLoad_Seatpost; maxLoad+=10){
    // inner loop records force vs displacement from 0 to maxLoad
    RightDown();
    for(i=10; i<=maxLoad; i+=0.5){
      forceSetting = i;
      SetForce(i);
      ReadInputPins();
      PrintSeatpostFvD("FvD "+ String(i));
      delay(125);
      ReadInputPins();
      PrintSeatpostFvD("FvD "+ String(i));
    }

    SetForce(0);
    KillAll();
    delay(2000);
    ReadInputPins();
    delay(200);
    PrintSeatpostFvD("Neutral");
    PrintStatusToLCD("");
    if(abs(rightDucerPosInch-neutralPosInch) >= neutralPosThreshold){ // error out if neutral position has shifted more than the threshold
      errorMsg = "Neutral Position Error";
      errorFlag = true;
      break;
    }
  }
}// GenerateContinuousSeatpostFvD


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

    KillRelays();

    testRelays = false;
}// TestRelays


//------------------------------------------------------------------------
// sets pressure between 3 and 100 psi, using one of two linear interpolations based on inital calibration testing
// added interpolation between 0 and 3 just in case regulator has that range
int SetPressure(float pressure, int regulatorNumber){
    int retVal = 0;

// Adjust the pressure setting using the appropriate offset determined during calibration.
    if(regulatorNumber == 1){
      PSI1setting = pressure; // record the "desired" pressure setting before including the offset
      pressure += PSI1settingOffset;
    }
    if(regulatorNumber == 2){
      PSI2setting = pressure; // record the "desired" pressure setting before including the offset
      pressure += PSI2settingOffset;
    }

// Input Checking
    if(pressure<0) return 0;

// Adjust the DAC value based on previous calibration work
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

    if(regulatorNumber == 1){
        pinMode(DAC1, OUTPUT);
        analogWrite(DAC1, retVal);
        DAC1_bits = retVal;
    }
    else if(regulatorNumber == 2){
        pinMode(DAC2, OUTPUT);
        analogWrite(DAC2, retVal);
        DAC2_bits = retVal;
    }
    return retVal;
}// SetPressure


//------------------------------------------------------------------------
void TestMsg(String msg){
    Serial1.print(msg);
    lcd.clear();
    Serial1.print(msg);

    return;
}// TestMsg


//------------------------------------------------------------------------
float SetForce(float force){
    float p1 = force/pullArea;
    float p2 = force/pushArea;

    float retVal = 0;
    if(testMode==7){
        p1 = p1*mode7Ratio; // sets downforce to a % of up force (enough to pull off)
    }
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
    if(testMode!=1 && testMode!=5 && testMode!=6){
        PushDown();
        delay(200);
    }
    PushUp();
    delay(500);
    if(testMode==1 || testMode==5 && testMode!=6){ //when in mode 1, 5, or 6, want "neutral" to be a lower reference point than fully topped out.
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

    // if error, wait for error flag to be reset
    while(errorFlag){
        statusMsg = "Error";
        ReadInputPins();
        PrintStatusToLCD(errorMsg);
        if(webUpdateFlag) UpdateDashboard();
        delay(200);
    }

    // wait for pause button to be toggled off
    while(paused){
        statusMsg = "Paused";
        ReadInputPins();
        PrintStatusToLCD("Paused");
        if(webUpdateFlag) UpdateDashboard();
        RunCalibrations();// runs functions enabled through webhooks
        delay(200);
    }

    SetForce(forceSetting);
    lcd.clear();
    delay(500); // to allow force to reset before resuming testing
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
    else if(command.substring(0,8)=="twistFvD") {
        command = command.substring(8);
        int tempLoad = command.toInt();
        if(tempLoad==tempLoad) calibrateTwistLoad = tempLoad; // check if NaN
        else return -1;
        twistFvD = true;
        return calibrateTwistLoad;
    }
    else if(command=="calibratePressure"){
        calibratePressure = true;
        return 1;
    }
    else if(command=="calibrateAll"){
        twistFvD = true;
        calibrateInPhase = true;
        return 1;
    }
    else if(command=="calibrateWindow"){
        calibrateWindow = true;
        webUpdateConstants = true;
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
    else if(command.substring(0,8)=="twistFvD") {
        command = command.substring(8);
        int tempLoad = command.toInt();
        if(tempLoad==tempLoad) calibrateTwistLoad = tempLoad; // check if NaN
        else return -1;
        twistFvD = true;
        return calibrateTwistLoad;
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
    else if(command.substring(0,11)=="seatpostFvD"){
        command = command.substring(11);
        if(command.substring(0,1)=="*") {
          command = command.substring(1);
          iterateFvD = true;
        }
        else iterateFvD = false;

        int tempLoad = command.toInt();
        if(tempLoad==tempLoad) maxTestLoad_Seatpost = tempLoad; // check if NaN
        else return -1;
        seatpostFvD = true;
        return maxTestLoad_Seatpost;
    }
    else if(command.substring(0,15)=="contSeatpostFvD"){
        command = command.substring(15);
        if(command.substring(0,1)=="*") {
          command = command.substring(1);
          iterateFvD = true;
        }
        else iterateFvD = false;

        int tempLoad = command.toInt();
        if(tempLoad==tempLoad) maxTestLoad_Seatpost = tempLoad; // check if NaN
        else return -1;
        contSeatpostFvD = true;
        return maxTestLoad_Seatpost;
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
        if(paused){
          paused = false;
          statusMsg = "Running";
        }
        else {
          paused = true;
          statusMsg = "Paused";
        }
        return paused;
    }
    else if(command=="I2C"){
        if(useI2C) useI2C = false;
        else useI2C = true;
        return useI2C;
    }
    else if(command=="useZeroPSI"){
        if(useZeroPSI) useZeroPSI = false;
        else useZeroPSI = true;
        return useZeroPSI;
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
        webDashboardRefreshRate = command.toInt();
        return webDashboardRefreshRate;
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
    else if(command.substring(0,4)=="test"){
        command = command.substring(4);
        testNumber = command.toInt();
        return testNumber;
    }
    else if(command=="resetError"){
        errorFlag = false;
        errorMsg = "None";
        return 1;
    }
    else if(command.substring(0,4)=="disp"){
        command = command.substring(4);
        displayMode = command.toInt();
        return displayMode;
    }
    else if(command=="status"){
      return cycleCount;
    }
    else if(command=="kill"){
      KillRelays();
    }
    else if(command.substring(0,3)=="cal"){ //adjust strain gauge calibration
      command = command.substring(3);
      float tempCal = command.toFloat();
      if(tempCal==tempCal){
        SG_calibrationFactor += tempCal;
        scale.set_scale(SG_calibrationFactor); //Adjust to this calibration factor
      }
      else return -1;
      return SG_calibrationFactor;
    }
    else if(command.substring(0,4)=="tare"){
        TareScale();
        return 1;
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
  webUpdateConstants = true;
  return forceSetting;
}// WebSetForce


//------------------------------------------------------------------------
long WebSetTimeout(String tStr){

    // if 1st char is a "*", only update state 1 (up direction)
    if(tStr.substring(0,1)=="*"){
        stateTimeout[1] = tStr.substring(1).toInt();
    }
    else{
        stateTimeout[0] = tStr.toInt();
        stateTimeout[1] = tStr.toInt();
        stateTimeout[2] = tStr.toInt();
    }

    period = stateTimeout[1]+stateTimeout[2];
    return period;
}// WebSetTimeout


void TareScale(){
  scale.tare(); //Reset the scale to 0
}// TareScale


float FindPSISettingOffset(float targetForce, int regulatorNumber, int tempState){
  float tempPSISetting = 0;
  float tempPSISettingOffset = 0;
  float lastForce = 0;
  long testStartTime = millis();
  long currentTime = testStartTime;
  bool errorFree = true;

  float targetPSISetting = 0;
  float tempArea = 0;
  if(regulatorNumber == 1){
    tempArea = pullArea;
    PSI1settingOffset = 0; // reset value
  }
  if(regulatorNumber == 2){
    tempArea = pushArea;
    PSI2settingOffset = 0; // reset value
  }
  targetPSISetting = targetForce/tempArea; // determine target setting in order to set maximum deviation from desired setting
  float maxPSISetting = targetPSISetting * 1.35; // 35% allowance for deviation from desired setting (out-of-calibration calibration allowance for regulator)

  SetPressure(0,regulatorNumber);
  SetState(tempState);

  while(SG_measuredForceAbsolute < targetForce && errorFree){
    lastForce = SG_measuredForceAbsolute;
    SetPressure(tempPSISetting,regulatorNumber);
    tempPSISetting+=0.1;
    delay(20);
    ReadInputPins();
    currentTime = millis();

    // if(lastForce > SG_measuredForceAbsolute){
    //   errorMsg = "FindPSISettingOffset: force no longer increasing";
    //   errorFree = false;
    // }
    if((currentTime - testStartTime) > 60000){
      errorMsg = "FindPSISettingOffset timeout";
      errorFree = false;
    }
    if(tempPSISetting >= maxPSISetting){
      errorMsg = "PSI setting 35% over target. Check Regulator #" + String(regulatorNumber) + " Setting or Air Supply";
      errorFree = false;
    }
  }

  if(!errorFree){
    errorFlag = true;
    paused = true;
//  return 0; //tbd accepting error condtions for now. - even if error flag is set to true, it gets reset because of consecutive error check so it's not super meaningful.
  }

  //tbd deleteme this is a stupid correction because I can't figure out why it always overshoots the desired force.
  tempPSISetting *= 0.9;

  tempPSISettingOffset = tempPSISetting - targetPSISetting;

  return tempPSISettingOffset;

}// FindPSISettingOffset


//------------------------------------------------------------------------
void PrintSeatpostFvD(String stateStr){
  // Message Header "Time,SystemState,cycleCount,testNumber,ForceSet,PSI1set,PSI2set,pullPressurePSI,pushPressurePSI,DAC1_bits,leftDucerPosBits,rightDucerPosBits,leftDucerPosInch,rightDucerPosInch,lastNeutralLeft,lastNeutralRight,SG_measuredForceAbsolute"
  String msg = "";

  msg += String(Time.now()) + ",";
  msg += stateStr + ",";
  msg += String(forceSetting,2) + ",";
  msg += String(PSI1setting,2) + ","; //pull
  msg += String(rightDucerPosInch,3) + ",";
  msg += String(SG_measuredForceAbsolute,3);

  PrintMsg(msg);
}// PrintSeatpostFvD
