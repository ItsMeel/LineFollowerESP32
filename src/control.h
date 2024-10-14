#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include <config.h>
#include <commonDefinitions.h>
#include <hardware.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

uint32_t NextIterationMS = 0;

bool RunMotors = false;
bool EnableLogging = false;

float Kp = DEFAULT_KP;
float Ki = DEFAULT_KI;
float Kd = DEFAULT_KD;

float ErrorSensores;
float ErrorSensoresPrevio;
float Error;
float ErrorProporcional;
float ErrorIntegral;
float ErrorDiferencial;

#if SOFT_TURNS && SHARP_TURNS
  const float MAX_ERROR = 2 * ERROR_CAP;
#elif SOFT_TURNS || SHARP_TURNS
  const float MAX_ERROR = ERROR_CAP;
#else
  #error "Select at least SOFT_TURNS and/or SHARP_TURNS"
#endif

void calcularErrorSensores(){
  if(ValorSensorDer1 && ValorSensorIzq1){
    ErrorSensores = 0;
  }
  else if(ValorSensorDer1){
    ErrorSensores = 1;
  }
  else if(ValorSensorIzq1){
    ErrorSensores = -1;
  }
  else if(ValorSensorDer1 && ValorSensorDer2){
    ErrorSensores = 2;
  }
  else if(ValorSensorIzq1 && ValorSensorIzq2){
    ErrorSensores = -2;
  }
  else if(ValorSensorDer2){
    ErrorSensores = 3;
  }
  else if(ValorSensorIzq2){
    ErrorSensores = -3;
  }
  else if(ValorSensorDer2 && ValorSensorDer3){
    ErrorSensores = 4;
  }
  else if(ValorSensorIzq2 && ValorSensorIzq3){
    ErrorSensores = -4;
  }
  else if(ValorSensorDer3){
    ErrorSensores = 5;
  }
  else if(ValorSensorIzq3){
    ErrorSensores = -5;
  }
  else if(ValorSensorDer3 && ValorSensorDer4){
    ErrorSensores = 6;
  }
  else if(ValorSensorIzq3 && ValorSensorIzq4){
    ErrorSensores = -6;
  }
  else if(ValorSensorDer4){
    ErrorSensores = 7;
  }
  else if(ValorSensorIzq4){
    ErrorSensores = -7;
  }
}

void calcularErrorPID(){
  ErrorIntegral += Ki * ErrorSensores;
  ErrorDiferencial = Kd * (ErrorSensores - ErrorSensoresPrevio);
  ErrorProporcional = Kp * ErrorSensores;
  Error = ErrorProporcional + ErrorIntegral + ErrorDiferencial;
  ErrorSensoresPrevio = ErrorSensores;
  
  if(Error > MAX_ERROR){
    Error = MAX_ERROR;
  }
  else if (Error < -MAX_ERROR){
    Error = -MAX_ERROR;
  }
}

void ejecutarMotores(){
  if(Error == 0){
    setPWMMotorDer(PWM_MAX_MOTOR_DER);
    setPWMMotorIzq(PWM_MAX_MOTOR_IZQ);
  }

  #if SOFT_TURNS && SHARP_TURNS
    else if(Error >= 1){
      setPWMMotorDer(PWM_MAX_MOTOR_DER * (abs(Error) - 1), true);
      setPWMMotorIzq(PWM_MAX_MOTOR_IZQ);
    }
    else if(Error <= -1){
      setPWMMotorDer(PWM_MAX_MOTOR_DER);
      setPWMMotorIzq(PWM_MAX_MOTOR_IZQ * (abs(Error) - 1), true);
    }
    else if(Error > 0){
      setPWMMotorDer(PWM_MAX_MOTOR_DER * (1 - abs(Error)));
      setPWMMotorIzq(PWM_MAX_MOTOR_IZQ);
    }
    else if(Error < 0){
      setPWMMotorDer(PWM_MAX_MOTOR_DER);
      setPWMMotorIzq(PWM_MAX_MOTOR_IZQ * (1 - abs(Error)));
    }
  #elif SOFT_TURNS
    else if(Error > 0){
      setPWMMotorDer(PWM_MAX_MOTOR_DER * (1 - abs(Error)));
      setPWMMotorIzq(PWM_MAX_MOTOR_IZQ);
    }
    else if(Error < 0){
      setPWMMotorDer(PWM_MAX_MOTOR_DER);
      setPWMMotorIzq(PWM_MAX_MOTOR_IZQ * (1 - abs(Error)));
    }
  #elif SHARP_TURNS
    else if(Error > 0){
      setPWMMotorDer(PWM_MAX_MOTOR_DER * abs(Error), true);
      setPWMMotorIzq(PWM_MAX_MOTOR_IZQ);
    }
    else if(Error < 0){
      setPWMMotorDer(PWM_MAX_MOTOR_DER);
      setPWMMotorIzq(PWM_MAX_MOTOR_IZQ * abs(Error), true);
    }
  #endif
}

void resetErrors(){
  ErrorSensoresPrevio = 0;
  Error = 0;
  ErrorProporcional = 0;
  ErrorIntegral = 0;
  ErrorDiferencial = 0;
}

void resetPIDConstants(){
  Kp = DEFAULT_KP;
  Ki = DEFAULT_KI;
  Kd = DEFAULT_KD;
  resetErrors();
}

void getPIDConstants(){
  PIDCONSTANTS_STRUCT PIDConstants;
  PIDConstants.Kp = Kp;
  PIDConstants.Ki = Ki;
  PIDConstants.Kd = Kd;
  xQueueSendToBack(QueueGetPIDConstants, &PIDConstants, 0);
}

void sentPIDLog(){
  PIDLOGGING_STRUCT PIDLogging;
  PIDLogging.RunMotors = RunMotors;
  PIDLogging.Kp = Kp;
  PIDLogging.Ki = Ki;
  PIDLogging.Kd = Kd;
  PIDLogging.ErrorSensores = ErrorSensores;
  PIDLogging.ErrorSensoresPrevio = ErrorSensoresPrevio;
  PIDLogging.ErrorProporcional = ErrorProporcional;
  PIDLogging.ErrorIntegral = ErrorIntegral;
  PIDLogging.ErrorDiferencial = ErrorDiferencial;
  PIDLogging.Error = Error;
  xQueueSendToBack(QueuePIDLogging, &PIDLogging, 0);
}

void readQueue(){
  if(uxQueueMessagesWaiting(QueuePIDCommands)){
    PIDCOMMANDS_ENUM Command;
    xQueueReceive(QueuePIDCommands, &Command, 0);
    switch (Command){
      case PIDCOMMANDS_STOPMOTORS:
        RunMotors = false;
      break;

      case PIDCOMMANDS_RUNMOTORS:
        RunMotors = true;
      break;

      case PIDCOMMANDS_RESETPIDCONSTANTS:
        resetPIDConstants();
      break;

      case PIDCOMMANDS_RESETERROR:
        resetErrors();
      break;

      case PIDCOMMANDS_GETPIDCONSTANTS:
        getPIDConstants();
      break;

      case PIDCOMMANDS_ENABLEPIDLOGGING:
        EnableLogging = true;
      break;

      case PIDCOMMANDS_DISABLEPIDLOGGING:
        EnableLogging = false;
      break;

      default:
      break;
    }
  }

  if(uxQueueMessagesWaiting(QueueSetPIDConstants)){
    PIDCONSTANTS_STRUCT PIDConstants;
    xQueueReceive(QueueSetPIDConstants, &PIDConstants, 0);
    Kp = PIDConstants.Kp;
    Ki = PIDConstants.Ki;
    Kd = PIDConstants.Kd;
    resetErrors();
  }
}

void executePID(void * parameter){
  while (1){
    if(millis() > NextIterationMS){
      if(NextIterationMS == 0){
        NextIterationMS = millis();
      }

      NextIterationMS += LOOP_PERIOD_MS;

      readQueue();
      leerSensores();
      calcularErrorSensores();
      calcularErrorPID();

      if(RunMotors){
        ejecutarMotores();
      }
      else{
        setPWMMotorDer(0);
        setPWMMotorIzq(0);
      }

      if(EnableLogging){
        sentPIDLog();
      }
    }
    vTaskDelay(1);
  }
}

#endif