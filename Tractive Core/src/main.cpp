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

// TWAI
#define NUM_CAN_READS                   25          // the number of messages to read each time the CAN task is called
#define FCB_CONTROL_ADDR                0x00A       // critical data for FCB
#define FCB_DATA_ADDR                   0x00B       // sensor information for FCB
#define RCB_CONTROL_ADDR                0x00C       // critical data for RCB
#define RCB_DATA_ADDR                   0x00D       // sensor information for RCB 
#define RINE_CONTROL_ADDR               0x0C0       // motor command address 
#define RINE_MOTOR_INFO_ADDR            0x0A5       // get rinehart motor infromation 
#define RINE_VOLT_INFO_ADDR             0x0A7       // get rinehart voltage information
#define BMS_GEN_DATA_ADDR               0x6B0       // important BMS data
#define BMS_CELL_DATA_ADDR              0x6B2       // cell data

// tasks & timers
#define IO_UPDATE_INTERVAL              50000       // 0.05 seconds in microseconds
#define TWAI_UPDATE_INTERVAL            50000       // 0.05 seconds in microseconds
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
CarData carData = {
  // driving data
  .drivingData = {
    .readyToDrive = false,
    .enableInverter = false,
    .prechargeState = PRECHARGE_OFF,

    .imdFault = true,
    .bmsFault = true,

    .commandedTorque = 0,
    .currentSpeed = 0.0f,
    .driveDirection = false,
    .driveMode = ECO, 
  },

  // Battery Status
  .batteryStatus = {
    .batteryChargeState = 0,
    .busVoltage = 0,
    .rinehartVoltage = 0,
    .pack1Temp = 0.0f,
    .pack2Temp = 0.0f,
    .packCurrent = 0.0f,
    .minCellVoltage = 0.0f,
    .maxCellVoltage = 0.0f,
  },

  // Sensors
  .sensors = {
    .rpmCounterFR = 0,
    .rpmCounterFL = 0,
    .rpmCounterBR = 0,
    .rpmCounterBL = 0,
    .rpmTimeFR = 0,
    .rpmTimeFL = 0,
    .rpmTimeBR = 0,
    .rpmTimeBL = 0,

    .wheelSpeedFR = 0.0f,
    .wheelSpeedFL = 0.0f,
    .wheelSpeedBR = 0.0f,
    .wheelSpeedBL = 0.0f,

    .wheelHeightFR = 0.0f,
    .wheelHeightFL = 0.0f,
    .wheelHeightBR = 0.0f,
    .wheelHeightBL = 0.0f,

    .steeringWheelAngle = 0,

    .vicoreTemp = 0.0f,
    .pumpTempIn = 0.0f,
    .pumpTempOut = 0.0f,
  },

  // Inputs
  .inputs = {
    .pedal0 = 0,
    .pedal1 = 0,
    .brakeFront = 0,
    .brakeRear = 0,
    .brakeRegen = 0,
    .coastRegen = 0,
  },

  // Outputs
  .outputs = {
    .buzzerActive = false,
    .buzzerCounter = 0,
    .brakeLight = false,
    .fansActive = false,
    .pumpActive = false,
  }
};


// Hardware Timers
hw_timer_t *timer1 = NULL;
hw_timer_t *timer2 = NULL;
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

// tasks
void IOReadTask(void* pvParameters);
void IOWriteTask(void* pvParameters);
void TWAIReadTask(void* pvParameters);
void TWAIWriteTask(void* pvParameters);

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
  pinMode(PUMP_ENABLE_PIN, OUTPUT);

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

  // start timers
  if (setup.ioActive)
    timerAlarmEnable(timer1);
  if (setup.twaiActive)
    timerAlarmEnable(timer2);
  
  // ----------------------------------------------------------------------------------------- //


  // ------------------------------- Scheduler & Task Status --------------------------------- //
  Serial.printf("I/O UPDATE STATUS: %s\n", timerAlarmEnabled(timer1) ? "RUNNING" : "DISABLED");
  Serial.printf("TWAI UPDATE STATUS: %s\n", timerAlarmEnabled(timer2) ? "RUNNING" : "DISABLED");
  
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

  static uint8_t ucParameterToPassRead;
  TaskHandle_t xHandleRead = NULL;
  xTaskCreate(IOReadTask, "Read-IO", TASK_STACK_SIZE, &ucParameterToPassRead, tskIDLE_PRIORITY, &xHandleRead);

  static uint8_t ucParameterToPassWrite;
  TaskHandle_t xHandleWrite = NULL;
  xTaskCreate(IOWriteTask, "Write-IO", TASK_STACK_SIZE, &ucParameterToPassWrite, tskIDLE_PRIORITY, &xHandleWrite);
  
  portEXIT_CRITICAL_ISR(&timerMux);

  return;
}


/**
 * @brief callback function for creating a new CAN Update task
 * 
 * @param args arguments to be passed to the task
 */
void TWAICallback() 
{
  portENTER_CRITICAL_ISR(&timerMux);

  // Read TWAI
  static uint8_t ucParameterToPassRead;
  TaskHandle_t xHandleRead = NULL;
  xTaskCreate(TWAIReadTask, "Read-TWAI", TASK_STACK_SIZE, &ucParameterToPassRead, tskIDLE_PRIORITY, &xHandleRead);
  
  // Write TWAI
  static uint8_t ucParameterToPassWrite;
  TaskHandle_t xHandleWrite = NULL;
  xTaskCreate(TWAIWriteTask, "Write-TWAI", TASK_STACK_SIZE, &ucParameterToPassWrite, tskIDLE_PRIORITY, &xHandleWrite);

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
void IORead(void* pvParameters)
{

  // read pedals
  uint16_t tmpPedal1 = analogReadMilliVolts(PEDAL_1_PIN);
  tmpPedal1 = map(tmpPedal1, 290, 1425, PEDAL_MIN, PEDAL_MAX);                // (0.29V - 1.379V) | values found via testing
  tractiveCoreData.inputs.pedal1 = CalculateThrottleResponse(tmpPedal1);

  uint16_t tmpPedal0 = analogReadMilliVolts(PEDAL_0_PIN);                     //  (0.59V - 2.75V) | values found via testing
  tmpPedal0 = map(tmpPedal0, 575, 2810, PEDAL_MIN, PEDAL_MAX);
  tractiveCoreData.inputs.pedal0 = CalculateThrottleResponse(tmpPedal0);


  // brake pressure / pedal
  float tmpFrontBrake = analogReadMilliVolts(FRONT_BRAKE_PIN);
  tractiveCoreData.inputs.frontBrake = map(tmpFrontBrake, 265, 855, PEDAL_MIN, PEDAL_MAX);  // (0.26V - 0.855V) | values found via testing

  float tmpRearBrake = analogReadMilliVolts(REAR_BRAKE_PIN);
  tractiveCoreData.inputs.rearBrake = map(tmpRearBrake, 265, 855, PEDAL_MIN, PEDAL_MAX);  // (0.26V - 0.855V) | values found via testing

  // start button 
  if (digitalRead(START_BUTTON_PIN) == LOW) {
    tractiveCoreData.tractive.startStatus = true;
  }

  // drive mode button
  if (digitalRead(DRIVE_MODE_BUTTON_PIN) == LOW) {
    // increment drive mode
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


  // calculate commanded torque
  GetCommandedTorque();


  // debugging
  if (debugger.debugEnabled) {
    debugger.IO_data = carData;
    debugger.ioReadTaskCount++;
  }

  // end task
  vTaskDelete(NULL);
}


/**
 * @brief writes I/O
 * 
 * @param pvParameters parameters passed to task
 */
void IOWrite(void* pvParameters)
{
  // turn off wifi for ADC channel 2 to function
  esp_wifi_stop();

  // get brake position
  float tmpBrake = analogReadMilliVolts(BRAKE_PIN);
  carData.inputs.brakeFront = map(tmpBrake, 265, 855, PEDAL_MIN, PEDAL_MAX);  // (0.26V - 0.855V) | values found via testing

  // read pedal potentiometer 1
  uint16_t tmpPedal1 = analogReadMilliVolts(PEDAL_1_PIN);
  tmpPedal1 = map(tmpPedal1, 290, 1425, PEDAL_MIN, PEDAL_MAX);                // (0.29V - 1.379V) | values found via testing
  carData.inputs.pedal1 = CalculateThrottleResponse(tmpPedal1);

  // wcb connection LED would also be in here

  // turn wifi back on to re-enable esp-now connection to wheel board
  esp_wifi_start();

  // get pedal positions
  uint16_t tmpPedal0 = analogReadMilliVolts(PEDAL_0_PIN);                     //  (0.59V - 2.75V) | values found via testing
  tmpPedal0 = map(tmpPedal0, 575, 2810, PEDAL_MIN, PEDAL_MAX);                // remap throttle response to 0 - 255 range
  carData.inputs.pedal0 = CalculateThrottleResponse(tmpPedal0);

  // brake light logic 
  if (carData.inputs.brakeFront >= BRAKE_LIGHT_THRESHOLD) {
    carData.outputs.brakeLight = true;      // turn it on 
  }

  else {
    carData.outputs.brakeLight = false;     // turn it off
  }

  // update wheel ride height values
  carData.sensors.wheelHeightFR = analogRead(WHEEL_HEIGHT_FR_SENSOR);
  carData.sensors.wheelHeightFL = analogRead(WHEEL_HEIGHT_FL_SENSOR);

  // update steering wheel position
  carData.sensors.steeringWheelAngle = analogRead(STEERING_WHEEL_POT);

  // buzzer logic
  if (carData.outputs.buzzerActive)
  {
    digitalWrite(BUZZER_PIN, HIGH);
    carData.outputs.buzzerCounter++;

    if (carData.outputs.buzzerCounter >= (2 * (SENSOR_POLL_INTERVAL / 10000)))    // convert to activations per second and multiply by 2
    {
      // update buzzer state and turn off the buzzer
      carData.outputs.buzzerActive = false;
      carData.outputs.buzzerCounter = 0;                        // reset buzzer count
      digitalWrite(BUZZER_PIN, LOW);

      carData.drivingData.enableInverter = true;                // enable the inverter so that we can tell rinehart to turn inverter on
    }
  }

  // ready to drive button
  if (digitalRead(RTD_BUTTON_PIN) == LOW) {
    if (carData.drivingData.readyToDrive) {
      // turn on buzzer to indicate TSV is live
      carData.outputs.buzzerActive = true;
    }
  }

  // Ready to Drive LED
  if (carData.drivingData.readyToDrive) {
    digitalWrite(RTD_LED_PIN, HIGH);
  }
  else {
    digitalWrite(RTD_LED_PIN, LOW);
  }

  // BMS fault LED
  if (!carData.drivingData.bmsFault) {
    digitalWrite(BMS_LED_PIN, LOW);
  }
  else {
    digitalWrite(BMS_LED_PIN, HIGH);
  }

  // IMD fault LED
  if (!carData.drivingData.imdFault) {
    digitalWrite(IMD_LED_PIN, LOW);
  }
  else {
    digitalWrite(IMD_LED_PIN, HIGH);
  }


  // calculate commanded torque
  GetCommandedTorque();


  // debugging
  if (debugger.debugEnabled) {
    debugger.IO_data = carData;
    debugger.ioWriteTaskCount++;
  }

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

  // --- receive messages --- //

  // if rx queue is full clear it (this is bad, implement can message filtering)
  uint32_t alerts;
  twai_read_alerts(&alerts, pdMS_TO_TICKS(TWAI_BLOCK_DELAY));
  if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
    twai_clear_receive_queue();
  }

  // check for new messages in the CAN buffer
  for (int i = 0; i < NUM_CAN_READS; ++i) {
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
          carData.batteryStatus.rinehartVoltage = (tmp2 << 8) | tmp1;   // little endian combination: value = (byte2 << 8) | byte1;
        break;

        case FCB_CONTROL_ADDR:
          carData.drivingData.readyToDrive =  incomingMessage.data[0];
          carData.drivingData.imdFault = incomingMessage.data[1];
          carData.drivingData.bmsFault = incomingMessage.data[2];
        break;

        case FCB_DATA_ADDR:
          carData.sensors.wheelSpeedBR = incomingMessage.data[0];
          carData.sensors.wheelSpeedBL = incomingMessage.data[1];
          carData.sensors.wheelHeightBR = incomingMessage.data[2];
          carData.sensors.wheelHeightBL = incomingMessage.data[3];
        break;

        // BMS: general pack data
        case BMS_GEN_DATA_ADDR:
          // pack current
          tmp1 = incomingMessage.data[0]; 
          tmp2 = incomingMessage.data[1];
          carData.batteryStatus.packCurrent = (tmp1 << 8) | tmp2;   // big endian combination: value = (byte1 << 8) | byte2;

          // pack voltage
          tmp1 = incomingMessage.data[2];
          tmp2 = incomingMessage.data[3];
          carData.batteryStatus.busVoltage = ((tmp1 << 8) | tmp2) / 10;    // big endian combination: value = (byte1 << 8) | byte2;

          // state of charge
          carData.batteryStatus.batteryChargeState = incomingMessage.data[4];
        break;

        // BMS: cell data
        case BMS_CELL_DATA_ADDR:
          carData.batteryStatus.minCellVoltage = incomingMessage.data[0];
          carData.batteryStatus.maxCellVoltage = incomingMessage.data[1];
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

  twai_message_t rineOutgoingMessage;
  rineOutgoingMessage.identifier = RINE_CONTROL_ADDR;
  rineOutgoingMessage.flags = TWAI_MSG_FLAG_NONE;
  rineOutgoingMessage.data_length_code = 8;

  // build message
  rineOutgoingMessage.data[0] = carData.drivingData.commandedTorque & 0xFF;       // commanded torque is sent across two bytes
  rineOutgoingMessage.data[1] = carData.drivingData.commandedTorque >> 8;
  rineOutgoingMessage.data[2] = 0x00;                                             // speed command NOT USING
  rineOutgoingMessage.data[3] = 0x00;                                             // speed command NOT USING
  rineOutgoingMessage.data[4] = (uint8_t)(carData.drivingData.driveDirection);    // 1: forward | 0: reverse (we run in reverse!)
  rineOutgoingMessage.data[5] = (uint8_t)(carData.drivingData.enableInverter);    // enable inverter command
  rineOutgoingMessage.data[6] = (MAX_TORQUE * 10) & 0xFF;                         // this is the max torque value that rinehart will push
  rineOutgoingMessage.data[7] = (MAX_TORQUE * 10) >> 8;                           // rinehart expects 10x value spread across 2 bytes

  // queue message for transmission
  esp_err_t rineCtrlResult = twai_transmit(&rineOutgoingMessage, pdMS_TO_TICKS(TWAI_BLOCK_DELAY));


  // setup RCB message
  twai_message_t rcbOutgoingMessage;
  rcbOutgoingMessage.identifier = RCB_CONTROL_ADDR;
  rcbOutgoingMessage.flags = TWAI_MSG_FLAG_NONE;
  rcbOutgoingMessage.data_length_code = 8;

  // build message for RCB - control
  rcbOutgoingMessage.data[0] = (uint8_t)carData.outputs.brakeLight;
  rcbOutgoingMessage.data[1] = 0x00;
  rcbOutgoingMessage.data[2] = 0x02;
  rcbOutgoingMessage.data[3] = 0x03;
  rcbOutgoingMessage.data[4] = 0x04;
  rcbOutgoingMessage.data[5] = 0x05;
  rcbOutgoingMessage.data[6] = 0x06;
  rcbOutgoingMessage.data[7] = 0x07;

  // queue message for transmission
  esp_err_t rcbCtrlResult = twai_transmit(&rcbOutgoingMessage, pdMS_TO_TICKS(TWAI_BLOCK_DELAY));

  // debugging
  if (debugger.debugEnabled) {
    debugger.TWAI_rineCtrlResult = rineCtrlResult;
    debugger.TWAI_rcbCtrlResult = rcbCtrlResult;

    for (int i = 0; i < 8; ++i) {
      debugger.TWAI_rineCtrlOutgoingMessage[i] = rineOutgoingMessage.data[i];
    }

    for (int i = 0; i < 8; ++i) {
      debugger.TWAI_rcbCtrlOutgoingMessage[i] = rcbOutgoingMessage.data[i];
    }

    debugger.twaiWriteTaskCount++;
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


/**
 * @brief Get the Commanded Torque from pedal values
 */
void GetCommandedTorque()
{
  // get the pedal average
  int pedalAverage = (carData.inputs.pedal0 + carData.inputs.pedal1) / 2;

  // drive mode logic (values are 10x because that is the format for Rinehart)
  switch (carData.drivingData.driveMode)
  {
    case SLOW:  // runs at 50% power
      carData.drivingData.commandedTorque = map(pedalAverage, PEDAL_MIN, PEDAL_MAX, 0, (MAX_TORQUE * 10) * 0.50);
    break;

    case ECO:   // runs at 75% power
      carData.drivingData.commandedTorque = map(pedalAverage, PEDAL_MIN, PEDAL_MAX, 0, (MAX_TORQUE * 10) * 0.75);
    break;

    case FAST:  // runs at 100% power
      carData.drivingData.commandedTorque = map(pedalAverage, PEDAL_MIN, PEDAL_MAX, 0, (MAX_TORQUE * 10));
    break;
    
    // error state, set the mode to ECO
    default:
      // set the state to ECO for next time
      carData.drivingData.driveMode = ECO;

      // we don't want to send a torque command if we were in an undefined state
      carData.drivingData.commandedTorque = 0;
    break;
  }

  // --- safety checks --- //

  // rinehart voltage check
  if (carData.batteryStatus.rinehartVoltage < MIN_BUS_VOLTAGE) {
    carData.drivingData.enableInverter = false;
  }

  // pedal difference 
  int pedalDifference = carData.inputs.pedal0 - carData.inputs.pedal1;
  if (_abs(pedalDifference) > (PEDAL_MAX * 0.15)) {
    carData.drivingData.commandedTorque = 0;
  }
  
  // buffer overflow / too much torque somehow
  if ((carData.drivingData.commandedTorque > (MAX_TORQUE * 10)) || (carData.drivingData.commandedTorque < 0)) {
    carData.drivingData.commandedTorque = 0;
  }

  // if brake is engaged
  // if (carData.outputs.brakeLight) {
  //   carData.drivingData.commandedTorque = 0;
  // }

  // check if ready to drive
  if (!carData.drivingData.readyToDrive) {
    carData.drivingData.commandedTorque = 0;      // if not ready to drive then block all torque
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
  switch (carData.drivingData.driveMode)
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
  Serial.printf("Incoming RTD Status: %s\n", carData.drivingData.readyToDrive ? "true" : "false");
  Serial.printf("Incoming IMD Fault Status: %s\n", carData.drivingData.imdFault ? "cleared" : "fault state");
  Serial.printf("Incoming BMS Fault Status: %s\n", carData.drivingData.bmsFault ? "fault state" : "cleared");

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

  // INPUTS
  // pedal 0 & 1
  Serial.printf("Pedal 0: %d\tPedal 1: %d\n", debugger.IO_data.inputs.pedal0, debugger.IO_data.inputs.pedal1);	

  // brake 0 & 1
  Serial.printf("Brake Front: %d\tBrake Rear: %d\n", debugger.IO_data.inputs.brakeFront, debugger.IO_data.inputs.brakeRear);

  // brake regen
  Serial.printf("Brake Regen: %d\n", debugger.IO_data.inputs.brakeRegen);

  // coast regen
  Serial.printf("Coast Regen: %d\n", debugger.IO_data.inputs.coastRegen);

  // faults
  Serial.printf("Faults: IMD: %d | BMS: %d\n", carData.drivingData.imdFault, carData.drivingData.bmsFault);

  // rtd
  Serial.printf("Ready to Drive: %s\n", carData.drivingData.readyToDrive ? "READY" : "DEACTIVATED");

  // inverter
  Serial.printf("Inverter Enable: %s\n", carData.drivingData.enableInverter ? "ENABLED" : "DISABLED");

  // OUTPUTS
  Serial.printf("Buzzer Status: %s, Buzzer Counter: %d\n", debugger.IO_data.outputs.buzzerActive ? "On" : "Off", debugger.IO_data.outputs.buzzerCounter);

  Serial.printf("Commanded Torque: %d\n", carData.drivingData.commandedTorque);
  
  Serial.printf("Drive Mode: %d\n", (int)carData.drivingData.driveMode);

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