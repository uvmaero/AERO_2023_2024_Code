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
#define NUM_TWAI_READS                   25          // the number of messages to read each time the CAN task is called
#define RINE_MOTOR_INFO_ADDR            0x0A5       // get motor information from Rinehart 
#define RINE_VOLT_INFO_ADDR             0x0A7       // get rinehart electrical information
#define RINE_BUS_INFO_ADDR              0x0AA       // get rinehart relay information 
#define RINE_MOTOR_CONTROL_ADDR         0x0C0       // motor command address 
#define RINE_BUS_CONTROL_ADDR           0x0C1       // control rinehart relay states
#define BMS_GEN_DATA_ADDR               0x6B0       // important BMS data
#define BMS_CELL_DATA_ADDR              0x6B2       // cell data

// tasks & timers
#define IO_UPDATE_INTERVAL              50000       // 0.05 seconds in microseconds
#define TWAI_UPDATE_INTERVAL            50000       // 0.05 seconds in microseconds
#define PRECHARGE_UPDATE_INTERVAL       200000      // 0.2 seconds in microseconds
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
  // debug toggle
  .debugEnabled = ENABLE_DEBUG,
  .TWAI_debugEnabled = false,
  .IO_debugEnabled = false,
  .scheduler_debugEnable = true,

  // debug data
  .TWAI_rineCtrlResult = ESP_OK,
  .TWAI_rcbCtrlResult = ESP_OK,
  .TWAI_rineCtrlOutgoingMessage = {},
  .TWAI_rcbCtrlOutgoingMessage = {},

  .RCB_updateResult = ESP_OK,
  .RCB_updateMessage = {},

  .IO_data = {},

  // scheduler data
  .ioReadTaskCount = 0,
  .ioWriteTaskCount = 0,
  .twaiReadTaskCount = 0,
  .twaiWriteTaskCount = 0,
};


/**
 * @brief the dataframe that describes the entire state of the car
 * 
 */
TractiveCoreData tractiveCoreData = {
  // tractive data
  .tractive = {
    .readyToDrive = false,
    .startStatus = false,
    .enableInverter = false,

    .prechargeState = PRECHARGE_OFF,

    .rinehartVoltage = 0.0f,
    .commandedTorque = 0,

    .driveDirection = false,
    .driveMode = ECO,
    
    .currentSpeed = 0.0f,

    .coastRegen = 0,
    .brakeRegen = 0,
  },

  // sensor data
  .sensors = {
    .imdFault = true,
    .bmsFault = true,

    .coolingTempIn = 0.0f,
    .coolingTempOut = 0.0f,
    .vicoreTemp = 0.0f,

    .glvReading = 0.0f,
  },

  // inputs
  .inputs = {
    .pedal0 = 0,
    .pedal1 = 0,

    .frontBrake = 0,
    .rearBrake = 0,
  },

  // outputs
  .outputs = {
    .driveModeLED = ECO,

    .brakeLightEnable = false,

    .fansEnable = false,

    .buzzerEnable = false,
    .buzzerCounter = 0,
  },

  // orion
  .orion = {
    .batteryChargeState = 0,

    .busVoltage = 0,

    .packCurrent = 0.0f,

    .minCellVoltage = 0.0f,
    .maxCellVoltage = 0.0f,
    .minCellTemp = 0.0f,
    .maxCellTemp = 0.0f,
  },
};


// Hardware Timers
hw_timer_t *timer1 = NULL;
hw_timer_t *timer2 = NULL;
hw_timer_t *timer3 = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;


// TWAI
static const twai_general_config_t can_general_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TWAI_TX_PIN, (gpio_num_t)TWAI_RX_PIN, TWAI_MODE_NORMAL);
static const twai_timing_config_t can_timing_config = TWAI_TIMING_CONFIG_500KBITS();
static const twai_filter_config_t can_filter_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();


/*
===============================================================================================
                                    Function Declarations 
===============================================================================================
*/


// callbacks
void IOUpdateCallback();
void TWAIUpdateCallback();
void PrechargeCallback();

// tasks
void IOReadTask(void* pvParameters);
void IOWriteTask(void* pvParameters);
void TWAIReadTask(void* pvParameters);
void TWAIWriteTask(void* pvParameters);
void PrechargeTask(void* pvParameters);

// helpers
void GetCommandedTorque();
uint16_t CalculateThrottleResponse(uint16_t value);


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

  // -------------------------- initialize serial connection ------------------------ //
  Serial.begin(9600);
  Serial.printf("\n\n|--- STARTING SETUP ---|\n\n");

  // setup managment struct
  struct setup
  {
    bool ioActive = false;
    bool twaiActive = false;
  };
  setup setup;
  // -------------------------- initialize GPIO ------------------------------ //
  analogReadResolution(12);
  
  // inputs
  pinMode(PEDAL_0_PIN, INPUT);
  pinMode(PEDAL_1_PIN, INPUT);
  pinMode(FRONT_BRAKE_PIN, INPUT);
  pinMode(REAR_BRAKE_PIN, INPUT);

  pinMode(COAST_REGEN_PIN, INPUT);
  pinMode(BRAKE_REGEN_PIN, INPUT);

  pinMode(START_BUTTON_PIN, INPUT_PULLUP);
  pinMode(DRIVE_MODE_BUTTON_PIN, INPUT_PULLUP);

  // outputs
  pinMode(RTD_LED_PIN, OUTPUT);
  pinMode(DRIVE_MODE_LED_PIN, OUTPUT);
  pinMode(BMS_FAULT_LED_PIN, OUTPUT);
  pinMode(IMD_FAULT_LED_PIN, OUTPUT);
  pinMode(FANS_ACTIVE_LED_PIN, OUTPUT);
  pinMode(PUMP_ACTIVE_LED_PIN, OUTPUT);

  pinMode(FAN_ENABLE_PIN, OUTPUT);

  pinMode(BRAKE_LIGHT_PIN, OUTPUT);

  pinMode(BUZZER_PIN, OUTPUT);

  Serial.printf("GPIO INIT [ SUCCESS ]\n");
  setup.ioActive = true;
  // -------------------------------------------------------------------------- //


  // --------------------- initialize CAN Controller -------------------------- //
  // install CAN driver
  if(twai_driver_install(&can_general_config, &can_timing_config, &can_filter_config) == ESP_OK) {
    Serial.printf("TWAI DRIVER INSTALL [ SUCCESS ]\n");

    // start CAN bus
    if (twai_start() == ESP_OK) {
      Serial.printf("TWAI INIT [ SUCCESS ]\n");
      twai_reconfigure_alerts(TWAI_ALERT_ALL, NULL);

      setup.twaiActive = true;
    }

    else {
      Serial.printf("TWAI INIT [ FAILED ]\n");
    }
  }

  else {
    Serial.printf("TWAI DRIVER INSTALL [ FAILED ]\n");
  }
  // --------------------------------------------------------------------------- //


  // ---------------------- initialize timer interrupts --------------------- //
  // timer 1 - Sensor Update
  timer1 = timerBegin(0, 80, true);
  timerAttachInterrupt(timer1, &IOUpdateCallback, true);
  timerAlarmWrite(timer1, IO_UPDATE_INTERVAL, true);

  // timer 2 - TWAI Update
  timer2 = timerBegin(1, 80, true);
  timerAttachInterrupt(timer2, &TWAIUpdateCallback, true);
  timerAlarmWrite(timer2, TWAI_UPDATE_INTERVAL, true);

  // timer 3 - precharge 
  timer3 = timerBegin(2, 80, true);
  timerAttachInterrupt(timer2, &PrechargeCallback, true);
  timerAlarmWrite(timer2, PRECHARGE_UPDATE_INTERVAL, true);

  // start timers
  if (setup.ioActive) {
    timerAlarmEnable(timer1);   // io
    timerAlarmEnable(timer3);   // precharge
  }
  if (setup.twaiActive) {
    timerAlarmEnable(timer2);   // twai
  }
  // ----------------------------------------------------------------------------------------- //


  // ------------------------------- Scheduler & Task Status --------------------------------- //
  Serial.printf("I/O UPDATE STATUS: %s\n", timerAlarmEnabled(timer1) ? "RUNNING" : "DISABLED");
  Serial.printf("TWAI UPDATE STATUS: %s\n", timerAlarmEnabled(timer2) ? "RUNNING" : "DISABLED");
  Serial.printf("PRECHARGE UPDATE STATUS: %s\n", timerAlarmEnabled(timer3) ? "RUNNING" : "DISABLED");

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
 * @brief callback function for creating a new sensor poll task
 * 
 * @param args arguments to be passed to the task
 */
void IOUpdateCallback() 
{
  portENTER_CRITICAL_ISR(&timerMux);

  // inits 
  static uint8_t ucParameterToPassRead;
  TaskHandle_t xHandleRead = NULL;

  static uint8_t ucParameterToPassWrite;
  TaskHandle_t xHandleWrite = NULL;

  // queue tasks 
  xTaskCreate(IOReadTask, "Read-IO", TASK_STACK_SIZE, &ucParameterToPassRead, tskIDLE_PRIORITY, &xHandleRead);
  xTaskCreate(IOWriteTask, "Write-IO", TASK_STACK_SIZE, &ucParameterToPassWrite, tskIDLE_PRIORITY, &xHandleWrite);
  
  portEXIT_CRITICAL_ISR(&timerMux);

  return;
}


/**
 * @brief callback function for creating a new CAN Update task
 * 
 * @param args arguments to be passed to the task
 */
void TWAIUpdateCallback() 
{
  portENTER_CRITICAL_ISR(&timerMux);

  // inits
  static uint8_t ucParameterToPassRead;
  TaskHandle_t xHandleRead = NULL;

  static uint8_t ucParameterToPassWrite;
  TaskHandle_t xHandleWrite = NULL;

  // queue tasks 
  xTaskCreate(TWAIReadTask, "Read-TWAI", TASK_STACK_SIZE, &ucParameterToPassRead, tskIDLE_PRIORITY, &xHandleRead);
  xTaskCreate(TWAIWriteTask, "Write-TWAI", TASK_STACK_SIZE, &ucParameterToPassWrite, tskIDLE_PRIORITY, &xHandleWrite);

  portEXIT_CRITICAL_ISR(&timerMux);
  
  return;
}


/**
 * @brief 
 * 
 */
void PrechargeCallback() {
  portENTER_CRITICAL_ISR(&timerMux);
  
  // inits
  static uint8_t ucParameterToPass;
  TaskHandle_t xHandle = NULL;

  // queue task
  xTaskCreate(PrechargeTask, "Precharge-Update", TASK_STACK_SIZE, &ucParameterToPass, tskIDLE_PRIORITY, &xHandle);
  
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
  tractiveCoreData.inputs.rearBrake = map(tmpRearBrake, 265, 855, PEDAL_MIN, PEDAL_MAX);  // (0.26V - 0.855V) | values found via testing

  uint16_t brakeAverage = (tractiveCoreData.inputs.frontBrake + tractiveCoreData.inputs.rearBrake) / 2;
  if (brakeAverage >= BRAKE_LIGHT_THRESHOLD) {
    tractiveCoreData.outputs.brakeLightEnable = true;
  }
  else {
    tractiveCoreData.outputs.brakeLightEnable = false;
  }

  // start button 
  if (digitalRead(START_BUTTON_PIN) == LOW && tractiveCoreData.tractive.readyToDrive) {
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
  tractiveCoreData.sensors.coolingTempIn = map(tmpCoolingIn, 0, 2500, 0, 100);

  int tmpCoolingOut = analogReadMilliVolts(COOLING_OUT_TEMP_PIN);
  tractiveCoreData.sensors.coolingTempOut = map(tmpCoolingOut, 0, 2500, 0, 100);

  if (tractiveCoreData.sensors.coolingTempIn >= COOLING_ENABLE_THRESHOLD) {
    tractiveCoreData.outputs.fansEnable = true;
  }
  if (tractiveCoreData.sensors.coolingTempIn <= COOLING_DISABLE_THRESHOLD) {
    tractiveCoreData.outputs.fansEnable = false;
  }

  // // debugging
  // if (debugger.debugEnabled) {
  //   debugger.IO_data = tractiveCoreData;
  //   debugger.ioReadTaskCount++;
  // }

  // end task
  vTaskDelete(NULL);
}


/**
 * @brief writes I/O
 * 
 * @param pvParameters parameters passed to task
 */
void IOWriteTask(void* pvParameters)
{
  // brake light 
  if (tractiveCoreData.outputs.brakeLightEnable) {
    digitalWrite(BRAKE_LIGHT_PIN, HIGH);
  }
  else {
    digitalWrite(BRAKE_LIGHT_PIN, LOW);
  }

  // cooling
  if (tractiveCoreData.outputs.fansEnable) {
    digitalWrite(FAN_ENABLE_PIN, HIGH);
  }
  else {
    digitalWrite(FAN_ENABLE_PIN, LOW);
  }

  // buzzer
  if (tractiveCoreData.outputs.buzzerEnable) {
    digitalWrite(BUZZER_PIN, HIGH);
    tractiveCoreData.outputs.buzzerCounter++;

    if (tractiveCoreData.outputs.buzzerCounter >= (2 * (IO_UPDATE_INTERVAL / 10000)))    // convert to activations per second and multiply by 2
    {
      // update buzzer state and turn off the buzzer
      tractiveCoreData.outputs.buzzerEnable = false;
      tractiveCoreData.outputs.buzzerCounter = 0;                        // reset buzzer count
      digitalWrite(BUZZER_PIN, LOW);

      tractiveCoreData.tractive.enableInverter = true;                   // enable the inverter
    }
  }

  // fault leds
  if (tractiveCoreData.sensors.bmsFault) {
    digitalWrite(BMS_FAULT_LED_PIN, HIGH);
  }
  else {
    digitalWrite(BMS_FAULT_LED_PIN, LOW);
  }

  if (tractiveCoreData.sensors.imdFault) {
    digitalWrite(IMD_FAULT_LED_PIN, HIGH);
  }
  else {
    digitalWrite(IMD_FAULT_LED_PIN, LOW);
  }

  // drive mode led
  // implement this doing some rgb led stuff

  // cooling led
  if (tractiveCoreData.outputs.fansEnable) {
    digitalWrite(FANS_ACTIVE_LED_PIN, HIGH);
  }
  else {
    digitalWrite(FANS_ACTIVE_LED_PIN, LOW);
  }

  // ready to drive LED
  if (tractiveCoreData.tractive.readyToDrive) {
    digitalWrite(RTD_LED_PIN, HIGH);
  }
  else {
    digitalWrite(RTD_LED_PIN, LOW);
  }

  // // debugging
  // if (debugger.debugEnabled) {
  //   debugger.IO_data = carData;
  //   debugger.ioWriteTaskCount++;
  // }

  // end task
  vTaskDelete(NULL);
}


/**
 * @brief reads and writes to the CAN bus
 * 
 * @param pvParameters parameters passed to task
 */
void TWAIReadTask(void* pvParameters)
{
  // inits
  twai_message_t incomingMessage;
  uint8_t tmp1, tmp2;
  int id;

  // if rx queue is full clear it (this is bad, implement twai message filtering)
  uint32_t alerts;
  twai_read_alerts(&alerts, pdMS_TO_TICKS(TWAI_BLOCK_DELAY));
  if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
    twai_clear_receive_queue();
  }

  // check for new messages in the CAN buffer
  for (int i = 0; i < NUM_TWAI_READS; ++i) {
    if (twai_receive(&incomingMessage, pdMS_TO_TICKS(TWAI_BLOCK_DELAY)) == ESP_OK) { // if there are messages to be read
      id = incomingMessage.identifier;
      
      // parse out data
      switch (id) {
        // Rinehart: voltage information
        case RINE_VOLT_INFO_ADDR:
          // rinehart voltage is spread across the first 2 bytes
          tmp1 = incomingMessage.data[0];
          tmp2 = incomingMessage.data[1];

          // combine the first two bytes and assign that to the rinehart voltage
          tractiveCoreData.tractive.rinehartVoltage = (tmp2 << 8) | tmp1;   // little endian combination: value = (byte2 << 8) | byte1;
        break;

        // BMS: general pack data
        case BMS_GEN_DATA_ADDR:
          // pack current
          tmp1 = incomingMessage.data[0]; 
          tmp2 = incomingMessage.data[1];
          tractiveCoreData.orion.packCurrent = (tmp1 << 8) | tmp2;   // big endian combination: value = (byte1 << 8) | byte2;

          // pack voltage
          tmp1 = incomingMessage.data[2];
          tmp2 = incomingMessage.data[3];
          tractiveCoreData.orion.busVoltage = ((tmp1 << 8) | tmp2) / 10;    // big endian combination: value = (byte1 << 8) | byte2;

          // state of charge
          tractiveCoreData.orion.batteryChargeState = incomingMessage.data[4];
        break;

        // BMS: cell data
        case BMS_CELL_DATA_ADDR:
          tractiveCoreData.orion.minCellVoltage = incomingMessage.data[0];
          tractiveCoreData.orion.maxCellVoltage = incomingMessage.data[1];
        break;

        default:
        break;
      }
    }
  }

  // end task
  vTaskDelete(NULL);
}


/**
 * @brief writes to TWAI
 * 
 * @param pvParameters parameters passed to task
 */
void TWAIWriteTask(void* pvParameters)
{
  // inits
  bool sentStatus = false;

  // tractive system control message
  twai_message_t rinehartMessage;
  rinehartMessage.identifier = RINE_MOTOR_CONTROL_ADDR;
  rinehartMessage.flags = TWAI_MSG_FLAG_NONE;
  rinehartMessage.data_length_code = 8;

  // build message
  rinehartMessage.data[0] = tractiveCoreData.tractive.commandedTorque & 0xFF;     // commanded torque is sent across two bytes
  rinehartMessage.data[1] = tractiveCoreData.tractive.commandedTorque >> 8;
  rinehartMessage.data[2] = 0x00;                                                 // speed command NOT USING
  rinehartMessage.data[3] = 0x00;                                                 // speed command NOT USING
  rinehartMessage.data[4] = (uint8_t)(tractiveCoreData.tractive.driveDirection);  // 1: forward | 0: reverse (we run in reverse!)
  rinehartMessage.data[5] = (uint8_t)(tractiveCoreData.tractive.enableInverter);  // enable inverter command
  rinehartMessage.data[6] = (MAX_TORQUE * 10) & 0xFF;                             // this is the max torque value that rinehart will push
  rinehartMessage.data[7] = (MAX_TORQUE * 10) >> 8;                               // rinehart expects 10x value spread across 2 bytes

  // queue message for transmission
  esp_err_t rineCtrlResult = twai_transmit(&rinehartMessage, pdMS_TO_TICKS(TWAI_BLOCK_DELAY));


  // --- precharge messages --- // 
  twai_message_t prechargeMessage;
  prechargeMessage.identifier = RINE_BUS_CONTROL_ADDR;
  prechargeMessage.flags = TWAI_MSG_FLAG_NONE;
  prechargeMessage.data_length_code = 8;

  esp_err_t prechargeMessageResult;

  // build rinehart message based on precharge state
  switch (tractiveCoreData.tractive.prechargeState) {
    case PRECHARGE_OFF:
      // message is sent to rinehart to turn everything off
      prechargeMessage.data[0] = 0x01;          // parameter address. LSB
      prechargeMessage.data[1] = 0x00;          // parameter address. MSB
      prechargeMessage.data[2] = 0x01;          // Read / Write Mode (0 = read | 1 = write)
      prechargeMessage.data[3] = 0x00;          // N/A
      prechargeMessage.data[4] = 0x00;          // Data: ( 0: all off | 1: relay 1 on | 2: relay 2 on | 3: relay 1 & 2 on )
      prechargeMessage.data[5] = 0x55;          // 0x55 means external relay control
      prechargeMessage.data[6] = 0x00;          // N/A
      prechargeMessage.data[7] = 0x00;          // N/A
    break;

    // do precharge
    case PRECHARGE_ON:
      // message is sent to rinehart to turn on precharge relay
      // precharge relay is on relay 1 in Rinehart
      prechargeMessage.data[0] = 0x01;          // parameter address. LSB
      prechargeMessage.data[1] = 0x00;          // parameter address. MSB
      prechargeMessage.data[2] = 0x01;          // Read / Write Mode (0 = read | 1 = write)
      prechargeMessage.data[3] = 0x00;          // N/A
      prechargeMessage.data[4] = 0x01;          // Data: ( 0: all off | 1: relay 1 on | 2: relay 2 on | 3: relay 1 & 2 on )
      prechargeMessage.data[5] = 0x55;          // 0x55 means external relay control
      prechargeMessage.data[6] = 0x00;          // N/A
      prechargeMessage.data[7] = 0x00;          // N/A
    break;


    // precharge complete!
    case PRECHARGE_DONE:
      // message is sent to rinehart to turn everything on
      // Keep precharge relay on and turn on main contactor
      prechargeMessage.data[0] = 0x01;          // parameter address. LSB
      prechargeMessage.data[1] = 0x00;          // parameter address. MSB
      prechargeMessage.data[2] = 0x01;          // Read / Write Mode (0 = read | 1 = write)
      prechargeMessage.data[3] = 0x00;          // N/A
      prechargeMessage.data[4] = 0x03;          // Data: ( 0: all off | 1: relay 1 on | 2: relay 2 on | 3: relay 1 & 2 on )
      prechargeMessage.data[5] = 0x55;          // 0x55 means external relay control
      prechargeMessage.data[6] = 0x00;          // N/A
      prechargeMessage.data[7] = 0x00;          // N/A
    break;


    // error state
    case PRECHARGE_ERROR:
      // message is sent to rinehart to turn everything off
      prechargeMessage.data[0] = 0x01;          // parameter address. LSB
      prechargeMessage.data[1] = 0x00;          // parameter address. MSB
      prechargeMessage.data[2] = 0x01;          // Read / Write Mode (0 = read | 1 = write)
      prechargeMessage.data[3] = 0x00;          // N/A
      prechargeMessage.data[4] = 0x00;          // Data: ( 0: all off | 1: relay 1 on | 2: relay 2 on | 3: relay 1 & 2 on )
      prechargeMessage.data[5] = 0x55;          // 0x55 means external relay control
      prechargeMessage.data[6] = 0x00;          // N/A
      prechargeMessage.data[7] = 0x00;          // N/A
    break;
  }

  // queue rinehart message for transmission
  prechargeMessageResult = twai_transmit(&prechargeMessage, pdMS_TO_TICKS(TWAI_BLOCK_DELAY));

  // debugging
  if (debugger.debugEnabled) {
    debugger.TWAI_rineCtrlResult = rineCtrlResult;

    for (int i = 0; i < 8; ++i) {
      debugger.TWAI_rineCtrlOutgoingMessage[i] = rinehartMessage.data[i];
    }

    debugger.twaiWriteTaskCount++;
  }

  // end task
  vTaskDelete(NULL);
}


/**
 * @brief run precharge
 * 
 * @param pvParameters 
 */
void PrechargeTask(void* pvParameters) {
  // inits
  twai_message_t outgoingMessage;
  int result;

  // precharge state machine
  switch (tractiveCoreData.tractive.prechargeState) {

    // prepare for and start precharge
    case PRECHARGE_OFF:

      // set ready to drive state
      tractiveCoreData.tractive.readyToDrive = false;

      if (tractiveCoreData.sensors.imdFault == false && tractiveCoreData.sensors.bmsFault == false) {
        tractiveCoreData.tractive.prechargeState = PRECHARGE_ON;
      }

    break;

    // do precharge
    case PRECHARGE_ON:

      // set ready to drive state
      tractiveCoreData.tractive.readyToDrive = false;

      // ensure voltages are above correct values
      if ((tractiveCoreData.tractive.rinehartVoltage >= (tractiveCoreData.orion.busVoltage * PRECHARGE_FLOOR)) &&
      (tractiveCoreData.orion.busVoltage > MIN_BUS_VOLTAGE)) {
        tractiveCoreData.tractive.prechargeState = PRECHARGE_DONE;
      }

    break;


    // precharge complete!
    case PRECHARGE_DONE:

      // set ready to drive state
      tractiveCoreData.tractive.readyToDrive = true;

      // if rinehart voltage drops below battery, something's wrong, 
      if (tractiveCoreData.tractive.rinehartVoltage < MIN_BUS_VOLTAGE) {
        tractiveCoreData.tractive.prechargeState = PRECHARGE_ERROR;
      }

    break;


    // error state
    case PRECHARGE_ERROR:

      // ensure car cannot drive
      tractiveCoreData.tractive.readyToDrive = false;
      tractiveCoreData.tractive.commandedTorque = 0;

      // reset precharge cycle
      tractiveCoreData.tractive.prechargeState = PRECHARGE_OFF;

    break;
    

    // handle undefined behavior
    default:

      // if we've entered an undefined state, go to error mode
      tractiveCoreData.tractive.prechargeState = PRECHARGE_ERROR;

    break;
  }

  // // debugging 
  // if (debugger.debugEnabled) {
  //   debugger.prechargeState = tractiveCoreData.tractive.prechargeState;
  //   debugger.prechargeTaskCount++;
  // }

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


/**
 * @brief Get the Commanded Torque from pedal values
 */
void GetCommandedTorque()
{
  // get the pedal average
  int pedalAverage = (tractiveCoreData.inputs.pedal0 + tractiveCoreData.inputs.pedal1) / 2;

  // drive mode logic (values are 10x because that is the format for Rinehart)
  switch (tractiveCoreData.tractive.driveMode)
  {
    case SLOW:  // runs at 50% power
      tractiveCoreData.tractive.commandedTorque = map(pedalAverage, PEDAL_MIN, PEDAL_MAX, 0, (MAX_TORQUE * 10) * 0.50);
    break;

    case ECO:   // runs at 75% power
      tractiveCoreData.tractive.commandedTorque = map(pedalAverage, PEDAL_MIN, PEDAL_MAX, 0, (MAX_TORQUE * 10) * 0.75);
    break;

    case FAST:  // runs at 100% power
      tractiveCoreData.tractive.commandedTorque = map(pedalAverage, PEDAL_MIN, PEDAL_MAX, 0, (MAX_TORQUE * 10));
    break;
    
    // error state, set the mode to ECO
    default:
      // set the state to ECO for next time
      tractiveCoreData.tractive.driveMode = ECO;

      // we don't want to send a torque command if we were in an undefined state
      tractiveCoreData.tractive.commandedTorque = 0;
    break;
  }

  // --- safety checks --- //

  // rinehart voltage check
  if (tractiveCoreData.tractive.rinehartVoltage < MIN_BUS_VOLTAGE) {
    tractiveCoreData.tractive.enableInverter = false;
  }

  // pedal difference 
  int pedalDifference = tractiveCoreData.inputs.pedal0 - tractiveCoreData.inputs.pedal1;
  if (_abs(pedalDifference) > (PEDAL_MAX * 0.15)) {
    tractiveCoreData.tractive.commandedTorque = 0;
  }
  
  // buffer overflow / too much torque somehow
  if ((tractiveCoreData.tractive.commandedTorque > (MAX_TORQUE * 10)) || (tractiveCoreData.tractive.commandedTorque < 0)) {
    tractiveCoreData.tractive.commandedTorque = 0;
  }

  // if brake is engaged
  // if (carData.outputs.brakeLight) {
  //   tractiveCoreData.tractive.commandedTorque = 0;
  // }

  // check if ready to drive
  if (!tractiveCoreData.tractive.readyToDrive) {
    tractiveCoreData.tractive.commandedTorque = 0;      // if not ready to drive then block all torque
  }
} 


/**
 * @brief calculate throttle response of pedal
 * 
 * @param value the raw pedal value
 * @return uint16_t the commanded torque value
 */
uint16_t CalculateThrottleResponse(uint16_t value) 
{
  // inits
  float calculatedResponse = 0;
  float exponent = 0;

  // check for buffer overflow
  if ((value > PEDAL_MAX) || (value < PEDAL_MIN)) {
    return 0;
  }

  // account for deadband
  if (value < PEDAL_DEADBAND) {
    return 0;
  }

  // determine response curve based on drive mode
  switch (tractiveCoreData.tractive.driveMode)
  {
    case SLOW:
      exponent = 4.0;
      calculatedResponse = (pow(value, exponent)) / (pow(PEDAL_MAX, exponent) / PEDAL_MAX);
    break;

    case ECO:
      exponent = 2.0;
      calculatedResponse = (pow(value, exponent)) / (pow(PEDAL_MAX, exponent) / PEDAL_MAX);
    break;

    case FAST:
      exponent = 0.75;
      calculatedResponse = (pow(value, exponent)) / (pow(PEDAL_MAX, exponent) / PEDAL_MAX);
    break;
    
    // if we are in an undefined state, pedals should do nothing
    default:
      return 0;
    break;
  }

  // cast final calculated response to an int
  return (int)calculatedResponse;
}


/* 
===============================================================================================
                                    DEBUG FUNCTIONS
================================================================================================
*/


/**
 * @brief some nice in-depth debugging for TWAI
 * 
 */
void PrintTWAIDebug() {
  Serial.printf("\n--- START TWAI DEBUG ---\n\n");
  // bus alerts            
  Serial.printf("TWAI BUS Alerts:\n");
  uint32_t alerts;
  twai_read_alerts(&alerts, pdMS_TO_TICKS(100));
  if (alerts & TWAI_ALERT_TX_SUCCESS) {
    Serial.printf("TWAI ALERT: TX Success\n");
  }
  if (alerts & TWAI_ALERT_TX_FAILED) {
    Serial.printf("TWAI ALERT: TX Failed\n");
  }
  if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
    Serial.printf("TWAI ALERT: RX Queue Full\n");
  }
  if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
    Serial.printf("TWAI ALERT: Surpassed Error Warning Limit\n");
  }
  if (alerts & TWAI_ALERT_ERR_PASS) {
    Serial.printf("TWAI ALERT: Entered Error Passive state\n");
  }
  if (alerts & TWAI_ALERT_BUS_OFF) {
    Serial.printf("TWAI ALERT: Bus Off\n");
  }

  Serial.printf("\n");

  // incoming data
  Serial.printf("Incoming RTD Status: %s\n", tractiveCoreData.tractive.readyToDrive ? "true" : "false");
  Serial.printf("Incoming IMD Fault Status: %s\n", tractiveCoreData.sensors.imdFault ? "cleared" : "fault state");
  Serial.printf("Incoming BMS Fault Status: %s\n", tractiveCoreData.sensors.bmsFault ? "fault state" : "cleared");

  // sent status
  Serial.printf("Rine Ctrl Send Status: 0x%X\n", debugger.TWAI_rineCtrlResult);
  Serial.printf("RCB Ctrl Send Status: 0x%X\n", debugger.TWAI_rcbCtrlResult);

  // messages
  Serial.printf("\n");
  Serial.printf("Rine Ctrl Outgoing Message:\n");
  for (int i = 0; i < 8; ++i) {
    Serial.printf("Byte %d: %02X\t", i, debugger.TWAI_rineCtrlOutgoingMessage[i]);
  }

  Serial.printf("\n");

  Serial.printf("RCB Ctrl Outgoing Message:\n");
  for (int i = 0; i < 8; ++i) {
    Serial.printf("Byte %d: %02X\t", i, debugger.TWAI_rcbCtrlOutgoingMessage[i]);
  }

  Serial.printf("\n\n--- END TWAI DEBUG ---\n");
}


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
  // CAN
  if (debugger.TWAI_debugEnabled) {
      PrintTWAIDebug();
  }

  // I/O
  if (debugger.IO_debugEnabled) {
    PrintIODebug();
  }

  // Scheduler
  if (debugger.scheduler_debugEnable) {
    Serial.printf("read io: %d | write io: %d | read twai: %d | write twai: %d\n", debugger.ioReadTaskCount, debugger.ioWriteTaskCount, 
    debugger.twaiReadTaskCount, debugger.twaiWriteTaskCount);
  }
}