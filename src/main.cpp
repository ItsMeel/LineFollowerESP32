#include <Arduino.h>
#include <BluetoothSerial.h>
#include <freertos/FreeRTOS.h>
#include <config.h>
#include <control.h>

uint32_t NextIterationMS = 0;

void setup() {
  Serial.begin(UART_BAUDRATE);
  setupHardware();
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