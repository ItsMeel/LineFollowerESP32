#ifndef CONTROL_H
#define CONTROL_H

#include <Arduino.h>
#include <config.h>
#include <hardware.h>

uint64_t TiempoActual;
uint64_t TiempoPrevio;
uint64_t DiferenciaTiempo;

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
  TiempoActual = micros();
  DiferenciaTiempo = TiempoActual - TiempoPrevio;
  ErrorIntegral += KP * KI * ErrorSensores * DiferenciaTiempo;
  ErrorDiferencial = KP * KD * (ErrorSensores - ErrorSensoresPrevio) / DiferenciaTiempo;
  ErrorProporcional = KP * ErrorSensores;
  Error = ErrorProporcional + ErrorIntegral + ErrorDiferencial;
  ErrorSensoresPrevio = ErrorSensores;
  TiempoPrevio = TiempoActual;
  
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
#endif