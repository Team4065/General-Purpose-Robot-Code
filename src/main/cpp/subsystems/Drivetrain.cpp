/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/Drivetrain.h"


Drivetrain::Drivetrain(){}

void Drivetrain::Periodic(){}

void Drivetrain::SetP_velocity(double P){
    this->kP_velocity_ = P;
}

void Drivetrain::SetI_velocity(double I){
    this->kI_velocity_ = I;
}

void Drivetrain::SetD_velocity(double D){
    this->kD_velocity_ = D;
}

void Drivetrain::SetFF_velocity(double FF){
    this->kFF_velocity_ = FF;
}



void Drivetrain::SetP_position(double P){
    this->kP_position_ = P;
}

void Drivetrain::SetI_position(double I){
    this->kI_position_ = I;
}

void Drivetrain::SetD_position(double D){
    this->kD_position_ = D;
}

void Drivetrain::SetFF_position(double FF){
    this->kFF_position_ = FF;
}



void Drivetrain::SetMaxVelocity(double MaxVelocity){
    this->kMaxVelocity_ = MaxVelocity;
}

void Drivetrain::SetMaxAcceleration(double MaxAcceleration){
    this->kMaxAcceleration_ = MaxAcceleration;
}