#include "ClearCore.h"
#include <Arduino.h>
#include <CmdParser.hpp>
#include "EthernetHandler.h"

#define motor ConnectorM0
#define baudRate 115200
#define HANDLE_ALERTS (0)
#define HomeSwitch ConnectorDI6
#define STARTUP_HOMING_LED IO1
#define COMMAND_MOVE_LED IO2

#define HOME_POSITION 0
#define END_POSITION -17500 // negative is "forward"
#define GLOBAL_ACCEL_MAX 1000000 // Q: Can we assume the safe max accel == safe max decel?
#define GLOBAL_VEL_MAX 100000 // Q: is this right?

// TODO: check the math on these markers
const int startLineEmergencyMarker = (END_POSITION - HOME_POSITION) * 0.05;
const int startLine = (END_POSITION - HOME_POSITION) * 0.1;
const int finishLine = (END_POSITION - HOME_POSITION) * 0.9;
const int finishLineEmergencyMarker = (END_POSITION - HOME_POSITION) * 0.95; 
const int startupVelMax = 100;
const int startupAccelMax = 400;

char msgBuffer[MAX_PACKET_LENGTH];

void setup();
void loop();
void PrintAlerts();
void HandleAlerts();
void StartupHoming();
void EmergencyStopCheck();

PinStatus homePin;
CmdParser cmdParser;

bool MoveLinear(int position, int velMax, int accelMax);
bool MoveHome();

void setup()
{
  cmdParser.setOptKeyValue(true);

  pinMode(STARTUP_HOMING_LED, OUTPUT);
  pinMode(COMMAND_MOVE_LED, OUTPUT);

  MotorMgr.MotorInputClocking(MotorManager::CLOCK_RATE_NORMAL);
  MotorMgr.MotorModeSet(MotorManager::MOTOR_ALL, Connector::CPM_MODE_STEP_AND_DIR);
  motor.HlfbMode(MotorDriver::HLFB_MODE_HAS_BIPOLAR_PWM);
  motor.HlfbCarrier(MotorDriver::HLFB_CARRIER_482_HZ);

  motor.VelMax(startupVelMax);
  motor.AccelMax(startupAccelMax);
  motor.PositionRefSet(0);

  Serial.flush();
  Serial.begin(baudRate);
  uint32_t timeout = 5000;
  uint32_t startTime = millis();
  while (!Serial && millis() - startTime < timeout)
  {
    continue;
  }

  motor.EnableRequest(true);

  Serial.println("Motor Enabled");

  Serial.println("Waiting for HLFB...");
  while (motor.HlfbState() != MotorDriver::HLFB_ASSERTED && !motor.StatusReg().bit.AlertsPresent)
  {
    continue;
  }

  if (motor.StatusReg().bit.AlertsPresent)
  {
    Serial.println("Motor alert detected.");
    PrintAlerts();
    if (HANDLE_ALERTS)
    {
      HandleAlerts();
    }
    else
    {
      Serial.println("Enable automatic alert handling by setting HANDLE_ALERTS to 1.");
    }
    Serial.println("Enabling may not have completed as expected. Proceed with caution.");
    Serial.println();
  }
  else
  {
    Serial.println("Motor Ready");
  }

  StartupHoming();

  ethernetSetup();
}

void loop()
{
  if (getEthernetMessage(msgBuffer))
  {
    Serial.println(msgBuffer);
    if (cmdParser.parseCmd(msgBuffer) != CMDPARSER_ERROR)
    {
      Serial.print("Command: ");
      Serial.println(cmdParser.getCommand());

      if (cmdParser.equalCommand("move"))
      {
        int accel = atoi(cmdParser.getValueFromKey("accel"));
        Serial.print("accel: ");
        Serial.println(accel);

        int vel = atoi(cmdParser.getValueFromKey("vel"));
        Serial.print("vel: ");
        Serial.println(vel);

        int pos = atoi(cmdParser.getValueFromKey("pos"));
        Serial.print("pos: ");
        Serial.println(pos);

        if (cmdParser.equalValueFromKey("type", "linear"))
        {
          MoveLinear(pos, vel, accel);
        }
        else
        {
          Serial.println("move type not recognized");
        }
      }
      else if (cmdParser.equalCommand("home"))
      {
        MoveHome();
      }
      else
      {
          Serial.println("Command not recognized!");
      }
    }
    else
    {
      Serial.println("Could not parse command");
    }
  }
}

bool MoveLinear(int position, int velMax, int accelMax)
{
  digitalWrite(COMMAND_MOVE_LED, true);
  if (velMax == 0) velMax = 10000;
  velMax = min(velMax, GLOBAL_VEL_MAX);
  if (accelMax == 0) accelMax = 1000000;
  accelMax = min(accelMax, GLOBAL_ACCEL_MAX);

  motor.VelMax(velMax);
  motor.AccelMax(accelMax);

  Serial.println("Executing LINEAR move");
  Serial.print("position: ");
  Serial.println(position);
  Serial.print("velMax: ");
  Serial.println(velMax);
  Serial.print("accelMax: ");
  Serial.println(accelMax);

  motor.Move(position, MotorDriver::MOVE_TARGET_ABSOLUTE);

  Serial.println("Beginning move loop...");
  while ((!motor.StepsComplete() || motor.HlfbState() != MotorDriver::HLFB_ASSERTED) && !motor.StatusReg().bit.AlertsPresent)
  {
    EmergencyStopCheck();
  }

  if (motor.StatusReg().bit.AlertsPresent)
  {
    Serial.println("Motor alert detected.");
    PrintAlerts();
    return false;
  }
  else
  {
    Serial.println("Move Done");
    digitalWrite(COMMAND_MOVE_LED, false);
    return true;
  }
}

bool MoveHome()
{
  digitalWrite(COMMAND_MOVE_LED, true);
  const int position = startLine;
  const int velMax = 50000;
  const int accelMax = 10000;

  motor.VelMax(velMax);
  motor.AccelMax(accelMax);

  Serial.println("Executing HOME command");
  Serial.print("position: ");
  Serial.println(position);
  Serial.print("velMax: ");
  Serial.println(velMax);
  Serial.print("accelMax: ");
  Serial.println(accelMax);

  motor.Move(position, MotorDriver::MOVE_TARGET_ABSOLUTE);

  Serial.println("Moving.. Waiting for HLFB");
  while ((!motor.StepsComplete() || motor.HlfbState() != MotorDriver::HLFB_ASSERTED) && !motor.StatusReg().bit.AlertsPresent)
  {
    continue;
  }

  digitalWrite(COMMAND_MOVE_LED, false);

  if (motor.StatusReg().bit.AlertsPresent)
  {
    Serial.println("Motor alert detected.");
    PrintAlerts();
    return false;
  }
  else
  {
    Serial.println("Move Done");
    return true;
  }
}

void EmergencyStopCheck()
{
  int pos = motor.PositionRefCommanded();
  int vel = motor.VelocityRefCommanded();
    Serial.println("Initiated the emergency stop! Motor values at moment of stop:");
    Serial.print("motor.PositionRefCommanded() = ");
    Serial.println(pos);
    Serial.print("motor.VelocityRefCommanded() = ");
    Serial.println(vel);
    Serial.println();
    Serial.print("startLineEmergencyMarker set at ");
    Serial.println(startLineEmergencyMarker);
    Serial.print("finishLineEmergencyMarker set at ");
    Serial.println(finishLineEmergencyMarker);
    
  if ((pos > startLineEmergencyMarker || pos < finishLineEmergencyMarker) && vel > 0) // Note polarity reversed because negative is "forward"
  {
    // Conditions met for emergency stop
    motor.MoveStopDecel(GLOBAL_ACCEL_MAX);
    Serial.println("Initiated the emergency stop! Motor values at moment of stop:");
    Serial.print("motor.PositionRefCommanded() = ");
    Serial.println(pos);
    Serial.print("motor.VelocityRefCommanded() = ");
    Serial.println(vel);
    Serial.println();
    Serial.print("startLineEmergencyMarker set at ");
    Serial.println(startLineEmergencyMarker);
    Serial.print("finishLineEmergencyMarker set at ");
    Serial.println(finishLineEmergencyMarker);
  }
}

void PrintAlerts()
{
  // report status of alerts
  Serial.println("Alerts present: ");
  if (motor.AlertReg().bit.MotionCanceledInAlert)
  {
    Serial.println("    MotionCanceledInAlert ");
  }
  if (motor.AlertReg().bit.MotionCanceledPositiveLimit)
  {
    Serial.println("    MotionCanceledPositiveLimit ");
  }
  if (motor.AlertReg().bit.MotionCanceledNegativeLimit)
  {
    Serial.println("    MotionCanceledNegativeLimit ");
  }
  if (motor.AlertReg().bit.MotionCanceledSensorEStop)
  {
    Serial.println("    MotionCanceledSensorEStop ");
  }
  if (motor.AlertReg().bit.MotionCanceledMotorDisabled)
  {
    Serial.println("    MotionCanceledMotorDisabled ");
  }
  if (motor.AlertReg().bit.MotorFaulted)
  {
    Serial.println("    MotorFaulted ");
  }
}

void HandleAlerts()
{
  if (motor.AlertReg().bit.MotorFaulted)
  {
    // if a motor fault is present, clear it by cycling enable
    Serial.println("Faults present. Cycling enable signal to motor to clear faults.");
    motor.EnableRequest(false);
    Delay_ms(10);
    motor.EnableRequest(true);
  }
  // clear alerts
  Serial.println("Clearing alerts.");
  motor.ClearAlerts();
}

bool MoveAtVelocity(long velocity)
{
  // Check if an alert is currently preventing motion
  if (motor.StatusReg().bit.AlertsPresent)
  {
    Serial.print("Motor status: 'In Alert'. Move Canceled.");
    return false;
  }

  motor.MoveVelocity(velocity);
  // SerialPort.SendLine("Ramping to speed...");
  while (!motor.StatusReg().bit.AtTargetVelocity)
  {
    continue;
  }

  return true;
}

bool IsAtHome()
{
  return HomeSwitch.State(); // note inversion
}

void StartupHoming()
{
  digitalWrite(STARTUP_HOMING_LED, true);
  // used to log progress periodically while waiting on movements or sensors
  int logFrequency = 1000; // ms
  uint32_t LogTime = Milliseconds() + logFrequency;
  MoveAtVelocity(startupVelMax);

  while (!IsAtHome())
  {
    if (Milliseconds() > LogTime)
    {
      Serial.println("Looking for home.");
      LogTime = Milliseconds() + logFrequency;
    }
  }

  // decelerate to stop
  motor.MoveStopDecel(startupAccelMax);
  Serial.println("Homing state: Found HOME SENSOR.");

  MoveAtVelocity(-50); // edge away from home; negative is away
  Serial.println("Homing state: Slowly moving above HOME SENSOR.");

  // waiting... for fallen edge of IsAtHome()
  while (IsAtHome())
  {
    if (Milliseconds() > LogTime)
    {
      Serial.println("Rising and still home.");
      LogTime = Milliseconds() + logFrequency;
    }
  }

  motor.MoveStopAbrupt();
  Serial.println("Stopping above HOME SENSOR.");

  motor.PositionRefSet(HOME_POSITION);
  Delay_ms(500);

  digitalWrite(STARTUP_HOMING_LED, false);
}