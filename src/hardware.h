#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>
#include <config.h>

bool ValorSensorDer4;
bool ValorSensorDer3;
bool ValorSensorDer2;
bool ValorSensorDer1;
bool ValorSensorIzq1;
bool ValorSensorIzq2;
bool ValorSensorIzq3;
bool ValorSensorIzq4;

void setupHardware(){
  // Motor GPIO initialization
  pinMode(MOTOR_PIN_DER_STBY, OUTPUT);
  pinMode(MOTOR_PIN_DER_PWM, OUTPUT);
  pinMode(MOTOR_PIN_DER_IN1, OUTPUT);
  pinMode(MOTOR_PIN_DER_IN2, OUTPUT);
  pinMode(MOTOR_PIN_IZQ_STBY, OUTPUT);
  pinMode(MOTOR_PIN_IZQ_PWM, OUTPUT);
  pinMode(MOTOR_PIN_IZQ_IN1, OUTPUT);
  pinMode(MOTOR_PIN_IZQ_IN2, OUTPUT);

  // Line sensor GPIO initialization
  pinMode(SENSOR_PIN_DER_4, INPUT);
  pinMode(SENSOR_PIN_DER_3, INPUT);
  pinMode(SENSOR_PIN_DER_2, INPUT);
  pinMode(SENSOR_PIN_DER_1, INPUT);
  pinMode(SENSOR_PIN_IZQ_1, INPUT);
  pinMode(SENSOR_PIN_IZQ_2, INPUT);
  pinMode(SENSOR_PIN_IZQ_3, INPUT);
  pinMode(SENSOR_PIN_IZQ_4, INPUT);
  pinMode(SENSOR_PIN_LINE_LEDON, OUTPUT);

  // IR sart module GPIO initialization
  pinMode(STARTER_PIN_START, INPUT);
  pinMode(STARTER_PIN_STOP, INPUT);

  // Ledc PWM initialization
  ledcAttachPin(MOTOR_PIN_DER_PWM, PWM_CHANNEL_MOTOR_DER);
  ledcAttachPin(MOTOR_PIN_IZQ_PWM, PWM_CHANNEL_MOTOR_IZQ);

  ledcChangeFrequency(PWM_CHANNEL_MOTOR_DER, PWM_FREQUENCY, 8);
  ledcChangeFrequency(PWM_CHANNEL_MOTOR_IZQ, PWM_FREQUENCY, 8);

  // Motor GPIO and PWM initial values
  digitalWrite(MOTOR_PIN_DER_STBY, HIGH);
  digitalWrite(MOTOR_PIN_IZQ_STBY, HIGH);

  digitalWrite(MOTOR_PIN_DER_IN1, LOW);
  digitalWrite(MOTOR_PIN_DER_IN2, LOW);
  digitalWrite(MOTOR_PIN_IZQ_IN1, LOW);
  digitalWrite(MOTOR_PIN_IZQ_IN2, LOW);

  ledcWrite(PWM_CHANNEL_MOTOR_DER, PWM_MAX_MOTOR_DER);
  ledcWrite(PWM_CHANNEL_MOTOR_IZQ, PWM_MAX_MOTOR_IZQ);

  // Line sensor activation
  digitalWrite(SENSOR_PIN_LINE_LEDON, HIGH);
}

void leerSensores(){
  #if DIGITAL_SENSORS
    #if SENSOR_INVERT
      ValorSensorDer4 = !digitalRead(SENSOR_PIN_DER_4);
      ValorSensorDer3 = !digitalRead(SENSOR_PIN_DER_3);
      ValorSensorDer2 = !digitalRead(SENSOR_PIN_DER_2);
      ValorSensorDer1 = !digitalRead(SENSOR_PIN_DER_1);
      ValorSensorIzq1 = !digitalRead(SENSOR_PIN_IZQ_1);
      ValorSensorIzq2 = !digitalRead(SENSOR_PIN_IZQ_2);
      ValorSensorIzq3 = !digitalRead(SENSOR_PIN_IZQ_3);
      ValorSensorIzq4 = !digitalRead(SENSOR_PIN_IZQ_4);
    #else
      ValorSensorDer4 = digitalRead(SENSOR_PIN_DER_4);
      ValorSensorDer3 = digitalRead(SENSOR_PIN_DER_3);
      ValorSensorDer2 = digitalRead(SENSOR_PIN_DER_2);
      ValorSensorDer1 = digitalRead(SENSOR_PIN_DER_1);
      ValorSensorIzq1 = digitalRead(SENSOR_PIN_IZQ_1);
      ValorSensorIzq2 = digitalRead(SENSOR_PIN_IZQ_2);
      ValorSensorIzq3 = digitalRead(SENSOR_PIN_IZQ_3);
      ValorSensorIzq4 = digitalRead(SENSOR_PIN_IZQ_4);
    #endif
  #else
    #if SENSOR_INVERT
      ValorSensorDer4 = analogRead(SENSOR_PIN_DER_4) <= SENSOR_ANALOG_TRESHOLD;
      ValorSensorDer3 = analogRead(SENSOR_PIN_DER_3) <= SENSOR_ANALOG_TRESHOLD;
      ValorSensorDer2 = analogRead(SENSOR_PIN_DER_2) <= SENSOR_ANALOG_TRESHOLD;
      ValorSensorDer1 = analogRead(SENSOR_PIN_DER_1) <= SENSOR_ANALOG_TRESHOLD;
      ValorSensorIzq1 = analogRead(SENSOR_PIN_IZQ_1) <= SENSOR_ANALOG_TRESHOLD;
      ValorSensorIzq2 = analogRead(SENSOR_PIN_IZQ_2) <= SENSOR_ANALOG_TRESHOLD;
      ValorSensorIzq3 = analogRead(SENSOR_PIN_IZQ_3) <= SENSOR_ANALOG_TRESHOLD;
      ValorSensorIzq4 = analogRead(SENSOR_PIN_IZQ_4) <= SENSOR_ANALOG_TRESHOLD;
    #else
      ValorSensorDer4 = analogRead(SENSOR_PIN_DER_4) >= SENSOR_ANALOG_TRESHOLD;
      ValorSensorDer3 = analogRead(SENSOR_PIN_DER_3) >= SENSOR_ANALOG_TRESHOLD;
      ValorSensorDer2 = analogRead(SENSOR_PIN_DER_2) >= SENSOR_ANALOG_TRESHOLD;
      ValorSensorDer1 = analogRead(SENSOR_PIN_DER_1) >= SENSOR_ANALOG_TRESHOLD;
      ValorSensorIzq1 = analogRead(SENSOR_PIN_IZQ_1) >= SENSOR_ANALOG_TRESHOLD;
      ValorSensorIzq2 = analogRead(SENSOR_PIN_IZQ_2) >= SENSOR_ANALOG_TRESHOLD;
      ValorSensorIzq3 = analogRead(SENSOR_PIN_IZQ_3) >= SENSOR_ANALOG_TRESHOLD;
      ValorSensorIzq4 = analogRead(SENSOR_PIN_IZQ_4) >= SENSOR_ANALOG_TRESHOLD;
    #endif
    
  #endif
}

void setPWMMotorDer(uint8_t Velocidad, bool InvGiro = false){
  if(Velocidad){
    digitalWrite(MOTOR_PIN_DER_STBY, HIGH);
  }
  else{
    digitalWrite(MOTOR_PIN_DER_STBY, LOW);
  }

  ledcWrite(PWM_CHANNEL_MOTOR_DER, Velocidad);

  if(InvGiro){
    digitalWrite(MOTOR_PIN_DER_IN1, HIGH);
    digitalWrite(MOTOR_PIN_DER_IN2, LOW);
  }
  else{
    digitalWrite(MOTOR_PIN_DER_IN1, LOW);
    digitalWrite(MOTOR_PIN_DER_IN2, HIGH);
  }
}

void setPWMMotorIzq(uint8_t Velocidad, bool InvGiro = false){
  if(Velocidad){
    digitalWrite(MOTOR_PIN_IZQ_STBY, HIGH);
  }
  else{
    digitalWrite(MOTOR_PIN_IZQ_STBY, LOW);
  }

  ledcWrite(PWM_CHANNEL_MOTOR_IZQ, Velocidad);

  if(InvGiro){
    digitalWrite(MOTOR_PIN_IZQ_IN1, HIGH);
    digitalWrite(MOTOR_PIN_IZQ_IN2, LOW);
  }
  else{
    digitalWrite(MOTOR_PIN_IZQ_IN1, LOW);
    digitalWrite(MOTOR_PIN_IZQ_IN2, HIGH);
  }
}
#endif
