#include <Arduino.h>

// Enable UART logging
#define SERIAL_LOGGING false

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
#define UART_BAUDRATE 115200

// PID Constants Kp0.5 Ki0.0001 Kd1
const float KP = 0.4;
const float KI = 0.00005;
const float KD = 3;
const float ERROR_CAP = 1.0; // Keep at 1.0

#define MOTOR_PIN_DER_STBY 19
#define MOTOR_PIN_DER_PWM 23
#define MOTOR_PIN_DER_IN1 22
#define MOTOR_PIN_DER_IN2 21
#define MOTOR_PIN_IZQ_STBY 16
#define MOTOR_PIN_IZQ_PWM 18
#define MOTOR_PIN_IZQ_IN1 5
#define MOTOR_PIN_IZQ_IN2 17

#define PWM_FREQUENCY 5000
#define PWM_CHANNEL_MOTOR_DER 0
#define PWM_CHANNEL_MOTOR_IZQ 1
#define PWM_MAX_MOTOR_DER 180
#define PWM_MAX_MOTOR_IZQ 180

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

#define STARTER_PIN_START 14
#define STARTER_PIN_STOP 13

uint64_t TiempoActual;
uint64_t TiempoPrevio;
uint64_t DiferenciaTiempo;

float ErrorSensores;
float ErrorSensoresPrevio;
float Error;
float ErrorProporcional;
float ErrorIntegral;
float ErrorDiferencial;

bool ValorSensorDer4;
bool ValorSensorDer3;
bool ValorSensorDer2;
bool ValorSensorDer1;
bool ValorSensorIzq1;
bool ValorSensorIzq2;
bool ValorSensorIzq3;
bool ValorSensorIzq4;

uint32_t NextIterationMS = 0;

#if SOFT_TURNS && SHARP_TURNS
  const float MAX_ERROR = 2 * ERROR_CAP;
#elif SOFT_TURNS || SHARP_TURNS
  const float MAX_ERROR = ERROR_CAP;
#else
  #error "Select at least SOFT_TURNS and/or SHARP_TURNS"
#endif

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
  /*DiferenciaTiempo = TiempoActual - TiempoPrevio;
  ErrorIntegral += KP * KI * ErrorSensores * DiferenciaTiempo;
  ErrorDiferencial = KP * KD * (ErrorSensores - ErrorSensoresPrevio) / DiferenciaTiempo;
  ErrorProporcional = KP * ErrorSensores;
  Error = ErrorProporcional + ErrorIntegral + ErrorDiferencial;
  ErrorSensoresPrevio = ErrorSensores;
  TiempoPrevio = TiempoActual;*/

  ErrorIntegral += KI * ErrorSensores;
  ErrorDiferencial = KD * (ErrorSensores - ErrorSensoresPrevio);
  ErrorProporcional = KP * ErrorSensores;
  Error = ErrorProporcional + ErrorIntegral + ErrorDiferencial;
  ErrorSensoresPrevio = ErrorSensores;
  
  if(Error > MAX_ERROR){
    Error = MAX_ERROR;
  }
  else if (Error < -MAX_ERROR){
    Error = -MAX_ERROR;
  }
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

void setup() {
  Serial.begin(UART_BAUDRATE);

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

void reportar(){
  Serial.print("I4: "); Serial.print(ValorSensorIzq4);
  Serial.print(" I3: "); Serial.print(ValorSensorIzq3);
  Serial.print(" I2: "); Serial.print(ValorSensorIzq2);
  Serial.print(" I1: "); Serial.print(ValorSensorIzq1);
  Serial.print(" D1: "); Serial.print(ValorSensorDer1);
  Serial.print(" D2: "); Serial.print(ValorSensorDer2);
  Serial.print(" D3: "); Serial.print(ValorSensorDer3);
  Serial.print(" D4: "); Serial.print(ValorSensorDer4);
  Serial.print(" ErrorSensores: "); Serial.print(ErrorSensores);
  Serial.print(" ErrorProporcional: "); Serial.print(ErrorProporcional);
  Serial.print(" ErrorIntegral: "); Serial.print(ErrorIntegral);
  Serial.print(" ErrorDerivativo: "); Serial.print(ErrorDiferencial);
  Serial.print(" Error: "); Serial.println(Error);
}

void loop() {
  if(millis() > NextIterationMS){
    if(NextIterationMS == 0){
      NextIterationMS = millis();
    }

    NextIterationMS += LOOP_PERIOD_MS;

    leerSensores();
    calcularErrorSensores();
    calcularErrorPID();

    #if RUN_MOTORS
      ejecutarMotores();
    #endif

    #if SERIAL_LOGGING
      reportar();
    #endif
  }
}