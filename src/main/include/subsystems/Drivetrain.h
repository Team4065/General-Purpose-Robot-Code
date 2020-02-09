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


class Drivetrain : public frc2::SubsystemBase {
 private:
   // Components (e.g. motor controllers and sensors) should generally be
   // declared private and exposed only through public methods.
   double kP_velocity_, kD_velocity_, kFF_velocity_ = 0;
   double kP_position_, kD_position_, kFF_position_ = 0;

 public:

   Drivetrain();

   virtual void Periodic();

   double leftTarget, rightTarget = 0.0;

   const double &kP_velocity = kP_velocity_;
   const double &kD_velocity = kD_velocity_;
   const double &kFF_velocity = kFF_velocity_;

   const double &kP_position = kP_position_;
   const double &kD_position = kD_position_;
   const double &kFF_position = kFF_position_;

   enum ControlMode {
      PERCENT,
      VELOCITY,
      POSITION
   } controlMode;

   enum PIDSlot{
      Velocity,
      Position
   };

   //Sets kP_velocity and updates the motorcontrollers
   virtual void SetP_velocity(double);
   //Sets kD_velocity and updates the motorcontrollers
   virtual void SetD_velocity(double);
   //Sets kFF_velocity and updates the motorcontrollers
   virtual void SetFF_velocity(double);

   //Sets kP_position and updates the motorcontrollers
   virtual void SetP_position(double);
   //Sets kD_position and updates the motorcontrollers
   virtual void SetD_position(double);
   //Sets kF_position and updates the motorcontrollers
   virtual void SetFF_position(double);
};

