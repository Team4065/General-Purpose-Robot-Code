/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include "subsystems/Drivetrain.h"

#include <rev/CANSparkMax.h>
#include <rev/CANPIDController.h>

class CANSparkMax_Drivetrain : public Drivetrain {
 public:
  CANSparkMax_Drivetrain(int leftMotorCount, int rightMotorCount, bool isForwardInverted = false, bool isBrushless = false);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic();

  //Sets kP_velocity and updates the motorcontrollers
  void SetP_velocity(double) override;
  //Sets kI_velocity and updates the motorcontrollers
  void SetI_velocity(double) override;
  //Sets kD_velocity and updates the motorcontrollers
  void SetD_velocity(double) override;
  //Sets kFF_velocity and updates the motorcontrollers
  void SetFF_velocity(double) override;

  //Sets kP_position and updates the motorcontrollers
  void SetP_position(double) override;
  //Sets kI_velocity and updates the motorcontrollers
  void SetI_position(double) override;
  //Sets kD_position and updates the motorcontrollers
  void SetD_position(double) override;
  //Sets kF_position and updates the motorcontrollers
  void SetFF_position(double) override;

 private:

  rev::CANSparkMax *leftMaster, *rightMaster; //These are the motor controllers that act as masters for the drivetrain
  rev::CANPIDController leftPID(),rightPID();  //These allow access to inbuilt closed loops in the motor controllers
  rev::CANSparkMax *leftSlaves[2], *rightSlaves[2];

  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.
};
