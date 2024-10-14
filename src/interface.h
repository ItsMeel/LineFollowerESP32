#ifndef INTERFACE_H
#define INTERFACE_H

#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_console.h>
#include "argtable3/argtable3.h"
#include <BluetoothSerial.h>
#include <commonDefinitions.h>
#include <config.h>

#define PROMPT_SIZE 512
#define MAX_PROMPT_ARGUMENTS 8

BluetoothSerial BTSerial;

static struct {
  struct arg_dbl *arg_kp;
  struct arg_dbl *arg_ki;
  struct arg_dbl *arg_kd;
  struct arg_end *end;
} setPIDConstants_args;

static struct {
  struct arg_end *end;
} runMotors_args;

static struct {
  struct arg_end *end;
} stopMotors_args;

static struct {
  struct arg_end *end;
} resetErrorPID_args;

static struct {
  struct arg_end *end;
} resetConstantsPID_args;

static struct {
  struct arg_end *end;
} getConstantsPID_args;

static struct {
  struct arg_end *end;
} enablePIDLogging_args;

static struct {
  struct arg_end *end;
} disablePIDLogging_args;

static int setPIDFunc(int argc, char **argv){
  arg_parse(argc, argv, (void **) &setPIDConstants_args);
  PIDCONSTANTS_STRUCT PIDConstants;
  PIDConstants.Kp = setPIDConstants_args.arg_kp->dval[0];
  PIDConstants.Ki = setPIDConstants_args.arg_ki->dval[0];
  PIDConstants.Kd = setPIDConstants_args.arg_kd->dval[0];
  xQueueSendToBack(QueueSetPIDConstants, &PIDConstants, 0);
  BTSerial.printf("PID constants changed to Kp: %f, Ki: %f, Kd: %f\n", PIDConstants.Kp, PIDConstants.Ki, PIDConstants.Kd);
  return 0;
}

static int runMotorsFunc(int argc, char **argv){
  PIDCOMMANDS_ENUM PIDCommand = PIDCOMMANDS_RUNMOTORS;
  xQueueSendToBack(QueuePIDCommands, &PIDCommand, 0);
  BTSerial.printf("Motors enabled\n");
  return 0;
}

static int stopMotorsFunc(int argc, char **argv){
  PIDCOMMANDS_ENUM PIDCommand = PIDCOMMANDS_STOPMOTORS;
  xQueueSendToBack(QueuePIDCommands, &PIDCommand, 0);
  BTSerial.printf("Motors disabled\n");
  return 0;
}

static int resetErrorPIDFunc(int argc, char **argv){
  PIDCOMMANDS_ENUM PIDCommand = PIDCOMMANDS_RESETERROR;
  xQueueSendToBack(QueuePIDCommands, &PIDCommand, 0);
  BTSerial.printf("PID errors resetted\n");
  return 0;
}

static int resetConstantsPIDFunc(int argc, char **argv){
  PIDCOMMANDS_ENUM PIDCommand = PIDCOMMANDS_RESETPIDCONSTANTS;
  xQueueSendToBack(QueuePIDCommands, &PIDCommand, 0);
  BTSerial.printf("PID constants resetted\n");
  return 0;
}

static int getConstantsPIDFunc(int argc, char **argv){
  PIDCOMMANDS_ENUM PIDCommand = PIDCOMMANDS_GETPIDCONSTANTS;
  xQueueSendToBack(QueuePIDCommands, &PIDCommand, 0);
  return 0;
}

static int enablePIDLoggingFunc(int argc, char **argv){
  PIDCOMMANDS_ENUM PIDCommand = PIDCOMMANDS_ENABLEPIDLOGGING;
  xQueueSendToBack(QueuePIDCommands, &PIDCommand, 0);
  BTSerial.printf("PID logging enabled\n");
  return 0;
}

static int disablePIDLoggingFunc(int argc, char **argv){
  PIDCOMMANDS_ENUM PIDCommand = PIDCOMMANDS_DISABLEPIDLOGGING;
  xQueueSendToBack(QueuePIDCommands, &PIDCommand, 0);
  BTSerial.printf("PID logging desabled\n");
  return 0;
}

void setupBTInterface(){
  BTSerial.begin(BLUETOOTH_NAME);
  BTSerial.setPin(BLUETOOTH_PASSWORD);
  esp_console_config_t console_config = {
    .max_cmdline_length = PROMPT_SIZE,
    .max_cmdline_args = MAX_PROMPT_ARGUMENTS
  };
  esp_console_init(&console_config);
}

void registerCommands(){
  setPIDConstants_args.arg_kp = arg_dbl1(NULL, NULL, "<Kp>", "The proportional constant");
  setPIDConstants_args.arg_ki = arg_dbl1(NULL, NULL, "<Ki>", "The integral constant");
  setPIDConstants_args.arg_kd = arg_dbl1(NULL, NULL, "<Kd>", "The derivative constant");
  setPIDConstants_args.end = arg_end(4);

  runMotors_args.end = arg_end(1);

  stopMotors_args.end = arg_end(1);

  resetErrorPID_args.end = arg_end(1);

  resetConstantsPID_args.end = arg_end(1);

  getConstantsPID_args.end = arg_end(1);

  enablePIDLogging_args.end = arg_end(1);

  disablePIDLogging_args.end = arg_end(1);

  const esp_console_cmd_t setPIDConstants_cmd = {
    .command = "setPIDconstants",
    .help = "Set the PID constants.\n"
    "Examples:\n"
    " setPIDconstants 3.2 0.001 0\n",
    .hint = NULL,
    .func = &setPIDFunc,
    .argtable = &setPIDConstants_args
  };
  const esp_console_cmd_t runMotors_cmd = {
    .command = "runMotors",
    .help = "Enable motors.\n"
    "Examples:\n"
    " runMotors\n",
    .hint = NULL,
    .func = &runMotorsFunc,
    .argtable = &runMotors_args
  };
  const esp_console_cmd_t stopMotors_cmd = {
    .command = "stopMotors",
    .help = "Disable motors.\n"
    "Examples:\n"
    " stopMotors\n",
    .hint = NULL,
    .func = &stopMotorsFunc,
    .argtable = &stopMotors_args
  };
  const esp_console_cmd_t resetErrorPID_cmd = {
    .command = "resetErrorPID",
    .help = "Set to 0 all errors in the PID.\n"
    "Examples:\n"
    " resetErrorPID\n",
    .hint = NULL,
    .func = &resetErrorPIDFunc,
    .argtable = &resetErrorPID_args
  };
  const esp_console_cmd_t resetConstantsPID_cmd = {
    .command = "resetConstantsPID",
    .help = "Set the PID constants to their default values.\n"
    "Examples:\n"
    " resetConstantsPID\n",
    .hint = NULL,
    .func = &resetConstantsPIDFunc,
    .argtable = &resetConstantsPID_args
  };
  const esp_console_cmd_t getConstantsPID_cmd = {
    .command = "getConstantsPID",
    .help = "Get PID constants.\n"
    "Examples:\n"
    " getConstantsPID\n",
    .hint = NULL,
    .func = &getConstantsPIDFunc,
    .argtable = &getConstantsPID_args
  };
  const esp_console_cmd_t enablePIDLogging_cmd = {
    .command = "enablePIDLogging",
    .help = "Enable PID logging.\n"
    "Examples:\n"
    " enablePIDLogging\n",
    .hint = NULL,
    .func = &enablePIDLoggingFunc,
    .argtable = &enablePIDLogging_args
  };
  const esp_console_cmd_t disablePIDLogging_cmd = {
    .command = "disablePIDLogging",
    .help = "Disable PID logging.\n"
    "Examples:\n"
    " disablePIDLogging\n",
    .hint = NULL,
    .func = &disablePIDLoggingFunc,
    .argtable = &disablePIDLogging_args
  };
  esp_console_cmd_register(&setPIDConstants_cmd);
  esp_console_cmd_register(&runMotors_cmd);
  esp_console_cmd_register(&stopMotors_cmd);
  esp_console_cmd_register(&resetErrorPID_cmd);
  esp_console_cmd_register(&resetConstantsPID_cmd);
  esp_console_cmd_register(&getConstantsPID_cmd);
  esp_console_cmd_register(&enablePIDLogging_cmd);
  esp_console_cmd_register(&disablePIDLogging_cmd);
  esp_console_register_help_command();
}

void executeBTInterface(void * parameter){
  while (1){
    if(BTSerial.available()){
      char Prompt[PROMPT_SIZE] = "";
      BTSerial.readBytesUntil('\n', Prompt, PROMPT_SIZE);
      int returnError;
      esp_err_t consoleError = esp_console_run(Prompt, &returnError);
      if (consoleError == ESP_ERR_NOT_FOUND) {
        BTSerial.printf("Unrecognized command\n");
      } else if (consoleError == ESP_ERR_INVALID_ARG) {
        BTSerial.printf("command was empty\n");
      } else if (consoleError == ESP_OK && returnError != ESP_OK) {
        BTSerial.printf("Command returned non-zero error code: 0x%x (%s)\n", returnError, esp_err_to_name(returnError));
      } else if (consoleError != ESP_OK) {
        BTSerial.printf("Internal error: %s\n", esp_err_to_name(consoleError));
      }
    }
    if(uxQueueMessagesWaiting(QueueGetPIDConstants)){
      PIDCONSTANTS_STRUCT PIDConstants;
      xQueueReceive(QueueGetPIDConstants, &PIDConstants, 0);
      BTSerial.printf("Kp: %f, Ki: %f, Kd: %f\n", PIDConstants.Kp, PIDConstants.Ki, PIDConstants.Kd);
    }
    if(uxQueueMessagesWaiting(QueuePIDLogging)){
      PIDLOGGING_STRUCT PIDLogging;
      xQueueReceive(QueuePIDLogging, &PIDLogging, 0);
      BTSerial.printf("RunMotors: %s, Kp: %f, Ki: %f, Kd: %f, ErrorSensores: %f, ErrorSensoresPrevio: %f, ErrorProporcional: %f, ErrorIntegral: %f, ErrorDiferencial: %f, Error: %f\n",
        (PIDLogging.RunMotors) ? "true" : "false",
        PIDLogging.Kp,
        PIDLogging.Ki,
        PIDLogging.Kd,
        PIDLogging.ErrorSensores,
        PIDLogging.ErrorSensoresPrevio,
        PIDLogging.ErrorProporcional,
        PIDLogging.ErrorIntegral,
        PIDLogging.ErrorDiferencial,
        PIDLogging.Error
      );
    }
    vTaskDelay(1);
  }
}
#endif