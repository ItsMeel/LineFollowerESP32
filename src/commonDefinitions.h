#ifndef commonDefinitions_H
#define commonDefinitions_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define QUEUE_PIDCONSTANTS_PACKET_SIZE 2
#define QUEUE_PIDCOMMANDS_PACKET_SIZE 5
#define QUEUE_PIDLOGGING_PACKET_SIZE 50

enum PIDCOMMANDS_ENUM: uint8_t{
  PIDCOMMANDS_STOPMOTORS,
  PIDCOMMANDS_RUNMOTORS,
  PIDCOMMANDS_RESETPIDCONSTANTS,
  PIDCOMMANDS_RESETERROR,
  PIDCOMMANDS_GETPIDCONSTANTS,
  PIDCOMMANDS_ENABLEPIDLOGGING,
  PIDCOMMANDS_DISABLEPIDLOGGING
};

struct PIDCONSTANTS_STRUCT{
  float Kp;
  float Ki;
  float Kd;
};

struct PIDLOGGING_STRUCT{
    bool RunMotors;
    float Kp;
    float Ki;
    float Kd;
    float ErrorSensores;
    float ErrorSensoresPrevio;
    float ErrorProporcional;
    float ErrorIntegral;
    float ErrorDiferencial;
    float Error;
};

QueueHandle_t QueueSetPIDConstants;
QueueHandle_t QueueGetPIDConstants;
QueueHandle_t QueuePIDCommands;
QueueHandle_t QueuePIDLogging;

#endif