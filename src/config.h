#ifndef CONFIG_H
#define CONFIG_H

// Enable UART logging
#define SERIAL_LOGGING false
#define UART_BAUDRATE 115200

// Enable motors
#define RUN_MOTORS true

// Use ttl levels for interpreting the sensors
#define DIGITAL_SENSORS false

// Invert the background and line color (Line: white -> true, black -> false)
#define SENSOR_INVERT false

/* 
  PID operation mode for making turns, can it be only one option or both
    SOFT_TURNS:
      Error -> 0, Both motors go forward
      Error -> ERROR_CAP, A motor stops while the other goes forward
    SHARP_TURNS:
      Error -> 0, Both motors go forward
      Error -> ERROR_CAP, A motor goes backward while the other goes forward
    SOFT_TURNS and SHARP_TURNS:
      Error -> 0, Both motors go forward
      Error -> ERROR_CAP, A motor stops while the other goes forward
      Error -> 2 * ERROR_CAP, A motor goes backward while the other goes forward
*/
#define SOFT_TURNS true
#define SHARP_TURNS true

// Misc macros
#define LOOP_PERIOD_MS 10

// PID Constants
const float DEFAULT_KP = 0.20;
const float DEFAULT_KI = 0.0;
const float DEFAULT_KD = 0.0;
const float ERROR_CAP = 1.0; // Keep at 1.0

// Motor GPIO
#define MOTOR_PIN_DER_STBY 19
#define MOTOR_PIN_DER_PWM 23
#define MOTOR_PIN_DER_IN1 22
#define MOTOR_PIN_DER_IN2 21
#define MOTOR_PIN_IZQ_STBY 16
#define MOTOR_PIN_IZQ_PWM 18
#define MOTOR_PIN_IZQ_IN1 5
#define MOTOR_PIN_IZQ_IN2 17

// Motor LedC PWM
#define PWM_FREQUENCY 5000
#define PWM_CHANNEL_MOTOR_DER 0
#define PWM_CHANNEL_MOTOR_IZQ 1
#define PWM_MAX_MOTOR_DER 180
#define PWM_MAX_MOTOR_IZQ 180

// Line sensor GPIO
#define SENSOR_PIN_DER_4 26
#define SENSOR_PIN_DER_3 35
#define SENSOR_PIN_DER_2 34 
#define SENSOR_PIN_DER_1 33
#define SENSOR_PIN_IZQ_1 32
#define SENSOR_PIN_IZQ_2 39
#define SENSOR_PIN_IZQ_3 36 
#define SENSOR_PIN_IZQ_4 25
#define SENSOR_PIN_LINE_LEDON 27
#define SENSOR_ANALOG_TRESHOLD 3900

// IR Starter GPIO
#define STARTER_PIN_START 14
#define STARTER_PIN_STOP 13

// Bluetooth
#define BLUETOOTH_NAME "SpeedyGonzales"
#define BLUETOOTH_PASSWORD ""

// FreeRTOS Tasks
#define EXECUTEPID_HEAP_SIZE 4096
#define EXECUTEPID_PRIORITY 1
#define EXECUTEPID_CORE 1

#define EXECUTEBTINTERFACE_HEAP_SIZE 4096
#define EXECUTEBTINTERFACE_PRIORITY 1
#define EXECUTEBTINTERFACE_CORE 0

#endif