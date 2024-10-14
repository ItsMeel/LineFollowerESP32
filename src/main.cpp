#include <Arduino.h>
#include <BluetoothSerial.h>
#include <freertos/FreeRTOS.h>
#include <config.h>
#include <control.h>
#include <interface.h>

void setup() {
  Serial.begin(UART_BAUDRATE);
  setupHardware();
  setupBTInterface();
  registerCommands();

  QueueSetPIDConstants = xQueueCreate(QUEUE_PIDCONSTANTS_PACKET_SIZE, sizeof(PIDCONSTANTS_STRUCT));
  QueueGetPIDConstants = xQueueCreate(QUEUE_PIDCONSTANTS_PACKET_SIZE, sizeof(PIDCONSTANTS_STRUCT));
  QueuePIDCommands = xQueueCreate(QUEUE_PIDCOMMANDS_PACKET_SIZE, sizeof(PIDCOMMANDS_ENUM));
  QueuePIDLogging = xQueueCreate(QUEUE_PIDLOGGING_PACKET_SIZE, sizeof(PIDLOGGING_STRUCT));
}

void loop() {
  xTaskCreatePinnedToCore(executePID, "executePID", EXECUTEPID_HEAP_SIZE, NULL, EXECUTEPID_PRIORITY, NULL, EXECUTEPID_CORE);
  xTaskCreatePinnedToCore(executeBTInterface, "executeBTInterface", EXECUTEBTINTERFACE_HEAP_SIZE, NULL, EXECUTEBTINTERFACE_PRIORITY, NULL, EXECUTEBTINTERFACE_CORE);

  vTaskDelete(NULL);
}