/**
 * @file main.cpp
 * @author dominic gasperini
 * @brief tractive core
 * @version 1.0
 * @date 2023-05-04
 * 
 * @copyright Copyright (c) 2023
 * 
 * @ref https://espressif-docs.readthedocs-hosted.com/projects/arduino-esp32/en/latest/libraries.html#apis
 */


/*
===============================================================================================
                                    Includes 
===============================================================================================
*/


#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "driver/twai.h"
#include "rtc.h"
#include "rtc_clk_common.h"

#include <data_types.h>
#include <pin_config.h>


/*
===============================================================================================
                                    Definitions
===============================================================================================
*/


// definitions
#define TIRE_DIAMETER                   20.0        // diameter of the vehicle's tires in inches
#define WHEEL_RPM_CALC_THRESHOLD        100         // the number of times the hall effect sensor is tripped before calculating vehicle speed
#define BRAKE_LIGHT_THRESHOLD           50          // the threshold that must be crossed for the brake to be considered active
#define PEDAL_MIN                       0           // minimum value the pedals can read as
#define PEDAL_MAX                       255         // maximum value a pedal can read as
#define PEDAL_DEADBAND                  15          // ~5% of PEDAL_MAX
#define MAX_TORQUE                      225         // MAX TORQUE RINEHART CAN ACCEPT, DO NOT EXCEED 230!!!
#define MIN_BUS_VOLTAGE                 150         // min bus voltage
#define COOLING_ENABLE_THRESHOLD        30          // in degrees C 
#define COOLING_DISABLE_THRESHOLD       25          // in degrees C
#define PRECHARGE_FLOOR                 0.8         // precentage of bus voltage rinehart should be at

// TWAI
#define NUM_TWAI_READS                  5           // the number of messages to read from RX FIFO the TWAI read task is called, 5 is max capacity of the queue!
#define RINE_MOTOR_INFO_ADDR            0x0A5       // get motor information from Rinehart 
#define RINE_VOLT_INFO_ADDR             0x0A7       // get rinehart electrical information
#define RINE_BUS_INFO_ADDR              0x0AA       // get rinehart relay information 
#define RINE_MOTOR_CONTROL_ADDR         0x0C0       // motor command address 
#define RINE_BUS_CONTROL_ADDR           0x0C1       // control rinehart relay states
#define BMS_GEN_DATA_ADDR               0x6B0       // important BMS data
#define BMS_CELL_DATA_ADDR              0x6B2       // cell data

// tasks & timers
#define IO_READ_INTERVAL                200000      // 0.1 seconds in microseconds
#define LORA_UPDATE_INTERVAL            200000      // 0.2 seconds in microseconds
#define SERIAL_UPDATE_INTERVAL          250000      // 0.25 seconds in microseconds
#define TASK_STACK_SIZE                 4096        // in bytes
#define TWAI_BLOCK_DELAY                100         // time to block to complete function call in FreeRTOS ticks (milliseconds)

// debug
#define ENABLE_DEBUG                    true       // master debug message control
#if ENABLE_DEBUG
  #define MAIN_LOOP_DELAY               1000        // delay in main loop
#else
  #define MAIN_LOOP_DELAY               1
#endif


/*
===============================================================================================
                                  Global Variables
===============================================================================================
*/


/**
 * @brief debugger structure used for organizing debug information
 * 
 */
Debugger debugger = {
  // // debug toggle
  // .debugEnabled = ENABLE_DEBUG,
  // .TWAI_debugEnabled = false,
  // .IO_debugEnabled = false,
  // .scheduler_debugEnable = true,

  // // TWAI data
  // .TWAI_rinehartCtrlResult = ESP_OK,
  // .TWAI_prechargeCtrlResult = ESP_OK,
  // .TWAI_rinehartCtrlMessage = {},
  // .TWAI_prechargeCtrlMessage = {},

  // // I/O data
  // .IO_data = {},

  // // precharge data
  // .prechargeState = PRECHARGE_OFF,

  // // scheduler data
  // .ioReadTaskCount = 0,
  // .ioWriteTaskCount = 0,
  // .twaiReadTaskCount = 0,
  // .twaiWriteTaskCount = 0,
  // .prechargeTaskCount = 0,
};


/**
 * @brief the dataframe that describes the entire state of the car
 * 
 */
TelemetryCoreData telemetryCoreData = {
  // // tractive data
  // .tractive = {
  //   .readyToDrive = false,
  //   .enableInverter = false,

  //   .prechargeState = PRECHARGE_OFF,

  //   .rinehartVoltage = 0.0f,
  //   .commandedTorque = 0,

  //   .driveDirection = false,    // forward is false | reverse is true (we run backwards)
  //   .driveMode = ECO,
    
  //   .currentSpeed = 0.0f,

  //   .coastRegen = 0,
  //   .brakeRegen = 0,
  // },

  // // sensor data
  // .sensors = {
  //   .imdFault = true,
  //   .bmsFault = true,

  //   .coolingTempIn = 0.0f,
  //   .coolingTempOut = 0.0f,
  //   .vicoreTemp = 0.0f,

  //   .glvReading = 0.0f,
  // },

  // // inputs
  // .inputs = {
  //   .pedal0 = 0,
  //   .pedal1 = 0,

  //   .frontBrake = 0,
  //   .rearBrake = 0,
  // },

  // // outputs
  // .outputs = {
  //   .driveModeLED = ECO,

  //   .brakeLightEnable = false,

  //   .fansEnable = false,

  //   .buzzerEnable = false,
  //   .buzzerCounter = 0,
  // },

  // // orion
  // .orion = {
  //   .batteryChargeState = 0,

  //   .busVoltage = 0,

  //   .packCurrent = 0.0f,

  //   .minCellVoltage = 0.0f,
  //   .maxCellVoltage = 0.0f,
  //   .minCellTemp = 0.0f,
  //   .maxCellTemp = 0.0f,
  // },
};


// Hardware Timers
hw_timer_t *timer1 = NULL;
hw_timer_t *timer2 = NULL;
hw_timer_t *timer3 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


// GPS


// IMU


// Raspberry Pi


/*
===============================================================================================
                                    Function Declarations 
===============================================================================================
*/


// callbacks
void IOReadCallback();
void LoRaUpdateCallback();
void SerialUpdateCallback();

// tasks
void IOReadTask(void* pvParameters);
void LoRaUpdateTask(void* pvParameters);
void SerialReadTask(void* pvParameters);
void SerialWriteTask(void* pvParameters);


/*
===============================================================================================
                                            Setup 
===============================================================================================
*/


void setup() {
  // set power configuration
  esp_pm_configure(&power_configuration);

  if (debugger.debugEnabled) {
    // delay startup by 3 seconds
    vTaskDelay(3000);
  }

  // ----------------------- initialize serial connection --------------------- //
  Serial.begin(9600);
  Serial.printf("\n\n|--- STARTING SETUP ---|\n\n");

  // setup managment struct
  struct setup
  {
    bool ioActive = false;
    bool gpsActive = false;
    bool imuActive = false;
    bool rpiComActive = false;
    bool lora = false;
  };
  setup setup;
  // -------------------------- initialize GPIO ------------------------------ //
  analogReadResolution(12);
  
  // inputs

  // outputs

  Serial.printf("GPIO INIT [ SUCCESS ]\n");
  setup.ioActive = true;
  // -------------------------------------------------------------------------- //


  // -------------------------- initialize GPS -------------------------------- //

  // -------------------------------------------------------------------------- //

  // -------------------------- initialize IMU -------------------------------- //

  // -------------------------------------------------------------------------- //


  // ---------------------- initialize timer interrupts --------------------- //
  // timer 1 - I/O Update
  timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer1, &IOReadCallback, true);
  timerAlarmWrite(timer1, IO_READ_INTERVAL, true);

  // timer 2 - LoRa Update
  timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer2, &LoRaUpdateCallback, true);
  timerAlarmWrite(timer2, LORA_UPDATE_INTERVAL, true);

  // timer 3 - Serial 
  timer3 = timerBegin(2, 80, true);
  timerAttachInterrupt(timer2, &SerialUpdateCallback, true);
  timerAlarmWrite(timer2, SERIAL_UPDATE_INTERVAL, true);

  // start timers
  if (setup.ioActive)
    timerAlarmEnable(timer1);
  if (setup.lora) 
    timerAlarmEnable(timer2);
  if (setup.gpsActive && setup.imuActive) {
    timerAlarmEnable(timer3);
  }

  // ----------------------------------------------------------------------------------------- //


  // ------------------------------- Scheduler & Task Status --------------------------------- //
  Serial.printf("I/O UPDATE STATUS: %s\n", timerAlarmEnabled(timer1) ? "RUNNING" : "DISABLED");
  Serial.printf("LORA UPDATE STATUS: %s\n", timerAlarmEnabled(timer2) ? "RUNNING" : "DISABLED");
  Serial.printf("SERIAL UPDATE STATUS: %s\n", timerAlarmEnabled(timer3) ? "RUNNING" : "DISABLED");

  // scheduler status
  if (xTaskGetSchedulerState() == 2) {
    Serial.printf("\nScheduler Status: RUNNING\n");

    // clock frequency
    rtc_cpu_freq_config_t clock_config;
    rtc_clk_cpu_freq_get_config(&clock_config);
    Serial.printf("CPU Frequency: %dMHz\n", clock_config.freq_mhz);
  }
  else {
    Serial.printf("\nScheduler STATUS: FAILED\nHALTING OPERATIONS");
    while (1) {};
  }
  Serial.printf("\n\n|--- END SETUP ---|\n\n");
  // ---------------------------------------------------------------------------------------- //
}


/*
===============================================================================================
                                    Callback Functions
===============================================================================================
*/


/**
 * @brief callback function for queueing I/O read and write tasks
 * 
 * @param args arguments to be passed to the task
 */
void IOReadCallback() 
{
  portENTER_CRITICAL_ISR(&timerMux);

  // inits 
  static uint8_t ucParameterToPass;
  TaskHandle_t xHandle = NULL;

  // queue task
  xTaskCreate(IOReadTask, "Read-IO", TASK_STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle);
  
  portEXIT_CRITICAL_ISR(&timerMux);

  return;
}


/*
===============================================================================================
                                FreeRTOS Task Functions
===============================================================================================
*/


/**
 * @brief reads I/O
 * 
 * @param pvParameters parameters passed to task
 */
void IOReadTask(void* pvParameters)
{
  // read pedals
  uint16_t tmpPedal1 = analogReadMilliVolts(PEDAL_1_PIN);
  tmpPedal1 = map(tmpPedal1, 290, 1425, PEDAL_MIN, PEDAL_MAX);                // (0.29V - 1.379V) | values found via testing
  tractiveCoreData.inputs.pedal1 = CalculateThrottleResponse(tmpPedal1);

  uint16_t tmpPedal0 = analogReadMilliVolts(PEDAL_0_PIN);                     //  (0.59V - 2.75V) | values found via testing
  tmpPedal0 = map(tmpPedal0, 575, 2810, PEDAL_MIN, PEDAL_MAX);
  tractiveCoreData.inputs.pedal0 = CalculateThrottleResponse(tmpPedal0);

  // calculate commanded torque
  GetCommandedTorque();

  // brake pressure / pedal
  float tmpFrontBrake = analogReadMilliVolts(FRONT_BRAKE_PIN);
  tractiveCoreData.inputs.frontBrake = map(tmpFrontBrake, 265, 855, PEDAL_MIN, PEDAL_MAX);  // (0.26V - 0.855V) | values found via testing

  float tmpRearBrake = analogReadMilliVolts(REAR_BRAKE_PIN);
  tractiveCoreData.inputs.rearBrake = map(tmpRearBrake, 265, 855, PEDAL_MIN, PEDAL_MAX);    // (0.26V - 0.855V) | values found via testing

  uint16_t brakeAverage = (tractiveCoreData.inputs.frontBrake + tractiveCoreData.inputs.rearBrake) / 2;
  if (brakeAverage >= BRAKE_LIGHT_THRESHOLD) {
    tractiveCoreData.outputs.brakeLightEnable = true;
  }
  else {
    tractiveCoreData.outputs.brakeLightEnable = false;
  }

  // start button 
  if ((digitalRead(START_BUTTON_PIN) == LOW) && tractiveCoreData.tractive.readyToDrive && !tractiveCoreData.tractive.enableInverter) {
    // only activate the buzzer is the inverter is not enabled, we don't need to repeat actions
    tractiveCoreData.outputs.buzzerEnable = true;
  }

  // drive mode button
  if (digitalRead(DRIVE_MODE_BUTTON_PIN) == LOW) {
    switch (tractiveCoreData.tractive.driveMode) {
    case SLOW:
      tractiveCoreData.tractive.driveMode = ECO;
      break;

    case ECO:
      tractiveCoreData.tractive.driveMode = FAST;
      break;

    case FAST:
      tractiveCoreData.tractive.driveMode = SLOW;
      break;

    default:
      tractiveCoreData.tractive.driveMode = ECO;
      break;
    }
  }

  // faults
  if (digitalRead(BMS_FAULT_PIN) == LOW) {
    tractiveCoreData.sensors.bmsFault = false;
  }
  else {
    tractiveCoreData.sensors.bmsFault = true;
  }

  if (digitalRead(IMD_FAULT_PIN) == LOW) {
    tractiveCoreData.sensors.imdFault = false;
  }
  else {
    tractiveCoreData.sensors.imdFault = true;
  }

  // cooling 
  int tmpCoolingIn = analogReadMilliVolts(COOLING_IN_TEMP_PIN);
  tractiveCoreData.sensors.coolingTempIn = map(tmpCoolingIn, 0, 2500, 0, 100);      // find thermistor values via testing 

  int tmpCoolingOut = analogReadMilliVolts(COOLING_OUT_TEMP_PIN);
  tractiveCoreData.sensors.coolingTempOut = map(tmpCoolingOut, 0, 2500, 0, 100);    // find thermistor values via testing 

  if (tractiveCoreData.sensors.coolingTempIn >= COOLING_ENABLE_THRESHOLD) {
    tractiveCoreData.outputs.fansEnable = true;
  }
  if (tractiveCoreData.sensors.coolingTempIn <= COOLING_DISABLE_THRESHOLD) {
    tractiveCoreData.outputs.fansEnable = false;
  }

  // debugging
  if (debugger.debugEnabled) {
    debugger.IO_data = tractiveCoreData;
    debugger.ioReadTaskCount++;
  }

  // end task
  vTaskDelete(NULL);
}


/*
===============================================================================================
                                    Main Loop
===============================================================================================
*/


/**
 * @brief 
 * 
 */
void loop()
{
  // everything is managed by RTOS, so nothing really happens here!
  vTaskDelay(MAIN_LOOP_DELAY);    // prevent watchdog from getting upset

  // debugging
  if (debugger.debugEnabled) {
    PrintDebug();
  }
}


/* 
===============================================================================================
                                    DEBUG FUNCTIONS
================================================================================================
*/


/**
 * @brief some nice in-depth debugging for I/O
 * 
 */
void PrintIODebug() {
  Serial.printf("\n--- START I/O DEBUG ---\n");

  // // INPUTS
  // // pedal 0 & 1
  // Serial.printf("Pedal 0: %d\tPedal 1: %d\n", debugger.IO_data.inputs.pedal0, debugger.IO_data.inputs.pedal1);	

  // // brake 0 & 1
  // Serial.printf("Brake Front: %d\tBrake Rear: %d\n", debugger.IO_data.inputs.brakeFront, debugger.IO_data.inputs.brakeRear);

  // // brake regen
  // Serial.printf("Brake Regen: %d\n", debugger.IO_data.inputs.brakeRegen);

  // // coast regen
  // Serial.printf("Coast Regen: %d\n", debugger.IO_data.inputs.coastRegen);

  // // faults
  // Serial.printf("Faults: IMD: %d | BMS: %d\n", tractiveCoreData.tractive.imdFault, tractiveCoreData.tractive.bmsFault);

  // // rtd
  // Serial.printf("Ready to Drive: %s\n", tractiveCoreData.tractive.readyToDrive ? "READY" : "DEACTIVATED");

  // // inverter
  // Serial.printf("Inverter Enable: %s\n", tractiveCoreData.tractive.enableInverter ? "ENABLED" : "DISABLED");

  // // OUTPUTS
  // Serial.printf("Buzzer Status: %s, Buzzer Counter: %d\n", debugger.IO_data.outputs.buzzerActive ? "On" : "Off", debugger.IO_data.outputs.buzzerCounter);

  // Serial.printf("Commanded Torque: %d\n", tractiveCoreData.tractive.commandedTorque);
  
  // Serial.printf("Drive Mode: %d\n", (int)tractiveCoreData.tractive.driveMode);

  Serial.printf("\n--- END I/O DEBUG ---\n");
}


/**
 * @brief manages toggle-able debug settings
 * 
 */
void PrintDebug() {
  // // CAN
  // if (debugger.TWAI_debugEnabled) {
  //     PrintTWAIDebug();
  // }

  // // I/O
  // if (debugger.IO_debugEnabled) {
  //   PrintIODebug();
  // }

  // // Scheduler
  // if (debugger.scheduler_debugEnable) {
  //   Serial.printf("read io: %d | write io: %d | read twai: %d | write twai: %d\n", debugger.ioReadTaskCount, debugger.ioWriteTaskCount, 
  //   debugger.twaiReadTaskCount, debugger.twaiWriteTaskCount);
  // }
}