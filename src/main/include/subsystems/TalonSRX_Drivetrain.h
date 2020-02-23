/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <frc2/command/SubsystemBase.h>


#include "subsystems/Drivetrain.h"

#include "ctre/phoenix/motorcontrol/can/WPI_TalonSRX.h"
#include "ctre/phoenix/motorcontrol/can/WPI_VictorSPX.h"
#include "ctre/phoenix/motorcontrol/can/BaseMotorController.h"

using namespace ctre::phoenix::motorcontrol::can;

/**
 * A drivetrain subsystem that uses TalonSRX as the motor controller.
 * VictorSPX can be used as slave controllers.
 * CAN IDs range from 1 to 6.
 * IDs 1 - 3: left motors
 * IDs 4 - 6: right motors
 * 
 * @param leftMotorCount The number of motors on the left side of the drivetrain.
 * @param rightMotorCount The number of motors on the right side of the drivetrain.
 * @param areSlavesVictorSPX True if the slaves are VictorSPXs.
 * @param invertForward Inverts the forward direction of the drivetrain and inverts encoders accordingly.
 * 
 */
class TalonSRX_Drivetrain : public Drivetrain {
 public:
  TalonSRX_Drivetrain(int leftMotorCount, int rightMotorCount, bool invertForward = false,  bool areSlavesVictorSPX = false);
  // TalonSRX_Drivetrain(int leftMotorCount, int rightMotorCount, bool invertForward,  bool areSlavesVictorSPX);

  void Periodic() override;
  
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
  WPI_TalonSRX *leftMaster, *rightMaster;  //The motor controllers that act as masters
  BaseMotorController *leftSlaves[2], *rightSlaves[2]; //The motor controllers that act as slaves
                                                        //Slaves follow the masters
};
