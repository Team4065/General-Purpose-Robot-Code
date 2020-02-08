/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

//TODO invert encoder reading so that forward is positive.
//TODO find the smallest amounts of includes necessary in order to decrease compile time (hopefully).

#pragma once
 

#include <vector>
#include <type_traits>

#include <frc2/command/SubsystemBase.h>

#include <ctre/Phoenix.h>
#include "rev/CANSparkMax.h"


template <class MotorControllerType>
class Drivetrain : public frc2::SubsystemBase {
 public:
 
 //Max motor count per side: 3
  Drivetrain(int leftMotorCount, int rightMotorCount, bool flipForwardDirection = false){
    for(int i = 0; i < leftMotorCount; ++i){
      leftMotors[i] = new MotorControllerType(1 + i);
    }

    for(int i = 0; i < leftMotorCount; ++i){
      rightMotors[i] = new MotorControllerType(3 + i);
    }

    this->leftMotorCount = leftMotorCount;
    this->rightMotorCount = rightMotorCount;

    for(int i = 1; i < leftMotorCount; ++i){
      leftMotors[i]->Follow(*leftMotors[0]);
    }

    for(int i = 1; i < leftMotorCount; ++i){
      rightMotors[i]->Follow(*rightMotors[0]);
    }

    leftMotors[0]->SetInverted(!flipForwardDirection);//Makes it move forward instead of spin
    rightMotors[0]->SetInverted(flipForwardDirection);

  }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic(){
    for(int i = 0; i < leftMotorCount; ++i){
      SetMotor(leftMotors[i], true);
    }

    for(int i = 0; i < leftMotorCount; ++i){
      SetMotor(rightMotors[i], false);
    }
  }

  double leftTarget = 0;
  double rightTarget = 0;

  enum ControlMode {
    PERCENT,
    VELOCITY,
    POSITION
  } controlMode;

  double kP_velocity, kD_velocity, kFF_velocity = 0;
  double kP_position, kD_position, kFF_position = 0;

  void SetVelocityPIDF(double kP, double kD, double kFF){
    kP_velocity = kP;
    kD_velocity = kD;
    kFF_velocity = kFF;

    for(int i = 0; i < leftMotorCount; ++i){
      UpdatePIDValues(leftMotors[i]);
    }

    for(int i = 0; i < leftMotorCount; ++i){
      UpdatePIDValues(rightMotors[i]);
    }
  }

  void SetPositionPIDF(double kP, double kD, double kFF){
    kP_velocity = kP;
    kD_velocity = kD;
    kFF_velocity = kFF;

    for(int i = 0; i < leftMotorCount; ++i){
      UpdatePIDValues(leftMotors[i]);
    }

    for(int i = 0; i < leftMotorCount; ++i){
      UpdatePIDValues(rightMotors[i]);
    }
  }

 private:

  unsigned short leftMotorCount, rightMotorCount;
  MotorControllerType* leftMotors[3];
  MotorControllerType* rightMotors[3];

  enum PIDSlot{
    Velocity,
    Position
  };


  //SetMotor variant for WPI_TalonSRX class
  template <class T = MotorControllerType>
  std::enable_if_t<std::is_same<T, WPI_TalonSRX>::value> SetMotor(MotorControllerType* motor, bool isLeft){

    double outputValue = 0;
    if(isLeft)
      outputValue = leftTarget;
    else
      outputValue = rightTarget;

    switch(controlMode){
      case ControlMode::PERCENT:
        motor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, outputValue);
        break;
      case ControlMode::VELOCITY:
        motor->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, outputValue);
        break;
      case ControlMode::POSITION:
        motor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, outputValue);
        break;
    }

  }

  //SetMotor variant for TalonSRX class
  template <class T = MotorControllerType>
  std::enable_if_t<std::is_same<T, TalonSRX>::value> SetMotor(MotorControllerType* motor, bool isLeft){

    double outputValue = 0;
    if(isLeft)
      outputValue = leftTarget;
    else
      outputValue = rightTarget;

    switch(controlMode){
      case ControlMode::PERCENT:
        motor->Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, outputValue);
        break;
      case ControlMode::VELOCITY:
        motor->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, outputValue);
        break;
      case ControlMode::POSITION:
        motor->Set(ctre::phoenix::motorcontrol::ControlMode::Position, outputValue);
        break;
    }

  }

  //SetMotor variant for CANSparkMax class
  template <class T = MotorControllerType>
  std::enable_if_t<std::is_same<T, rev::CANSparkMax>::value> SetMotor(MotorControllerType* motor, bool isLeft){

    double outputValue = 0;
    if(isLeft)
      outputValue = leftTarget;
    else
      outputValue = rightTarget;

    switch(controlMode){
      case ControlMode::PERCENT:
        motor->Set(outputValue);
        break;
      case ControlMode::VELOCITY:
        //motor->GetPIDController().SetReference(outputValue, );
        break;
      case ControlMode::POSITION:
        break;
    }

  }



  //UpdatePIDValues variant for WPI_TalonSRX class
  template <class T = MotorControllerType>
  std::enable_if_t<std::is_same<T, WPI_TalonSRX>::value> UpdatePIDValues(MotorControllerType* motor){
    motor->Config_kP(kP_velocity, PIDSlot::Velocity);
    motor->Config_kD(kD_velocity, PIDSlot::Velocity);
    motor->Config_kF(kFF_velocity, PIDSlot::Velocity);

    motor->Config_kP(kP_velocity, PIDSlot::Position);
    motor->Config_kD(kD_position, PIDSlot::Position);
    motor->Config_kF(kFF_position, PIDSlot::Position);
  }

  //UpdatePIDValues variant for TalonSRX class
  template <class T = MotorControllerType>
  std::enable_if_t<std::is_same<T, TalonSRX>::value> UpdatePIDValues(MotorControllerType* motor){
    motor->Config_kP(kP_velocity, PIDSlot::Velocity);
    motor->Config_kD(kD_velocity, PIDSlot::Velocity);
    motor->Config_kF(kFF_velocity, PIDSlot::Velocity);

    motor->Config_kP(kP_velocity, PIDSlot::Position);
    motor->Config_kD(kD_position, PIDSlot::Position);
    motor->Config_kF(kFF_position, PIDSlot::Position);
  }

  //UpdatePIDValues variant for CANSparkMax class
  template <class T = MotorControllerType>
  std::enable_if_t<std::is_same<T, rev::CANSparkMax>::value> UpdatePIDValues(MotorControllerType* motor){
    rev::CANPIDController PIDController = &motor->GetPIDController();
    PIDController.SetP(kP_velocity, PIDSlot::Velocity);
    PIDController.SetD(kD_velocity, PIDSlot::Velocity);
    PIDController.SetFF(kFF_velocity, PIDSlot::Velocity);
    
    PIDController.SetP(kP_velocity, PIDSlot::Position);
    PIDController.SetD(kD_position, PIDSlot::Position);
    PIDController.SetFF(kFF_position, PIDSlot::Position);
  }


};