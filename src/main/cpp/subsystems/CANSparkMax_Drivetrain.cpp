/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/CANSparkMax_Drivetrain.h"

CANSparkMax_Drivetrain::CANSparkMax_Drivetrain(int leftMotorCount, int rightMotorCount, bool isForwardInverted, bool isBrushless) {
    //error checking
    if(leftMotorCount == 0){
        throw "Left motor count must be greater than 0.";
        return;
    }

    if(rightMotorCount == 0){
        throw "Right motor count must be greater than 0.";
        return;
    }

    //Picks between brushed and brushless motors (ask the mechanical or electrical teams for that information)
    rev::CANSparkMax::MotorType motorType = (isBrushless) ? rev::CANSparkMax::MotorType::kBrushless : rev::CANSparkMax::MotorType::kBrushed;

    leftMaster = new rev::CANSparkMax(1, motorType);
    rightMaster = new rev::CANSparkMax(4, motorType);

    leftPID = &leftMaster->GetPIDController();
    rightPID = &rightMaster->GetPIDController();

    for(int i = 0; i < leftMotorCount - 1; ++i){//IDs range from 2 to 3
        leftSlaves[i] = new rev::CANSparkMax(i + 2, motorType);//makes the slaves
        leftSlaves[i]->Follow(*leftMaster);//binds the slaves to the masters
    }

    for(int i = 0; i < rightMotorCount - 1; ++i){//IDs range from 5 to 6
        rightSlaves[i] = new rev::CANSparkMax(i + 5, motorType);//makes the slaves
        rightSlaves[i]->Follow(*rightMaster);//binds the slaves to the masters
    }

    SetP_velocity(kP_velocity_);
    SetI_velocity(kI_velocity_);
    SetD_velocity(kD_velocity_);
    SetFF_velocity(kFF_velocity_);

    SetP_position(kP_position_);
    SetI_position(kI_position_);
    SetD_position(kD_position_);
    SetFF_position(kFF_position_);
}

// This method will be called once per scheduler run
void CANSparkMax_Drivetrain::Periodic() {}

void CANSparkMax_Drivetrain::SetP_velocity(double P){
    this->kP_velocity_ = P;
    this->leftPID->SetP(P, PIDSlot::Velocity);
    this->rightPID->SetP(P, PIDSlot::Velocity);
}

void CANSparkMax_Drivetrain::SetI_velocity(double I){
    this->kI_velocity_ = I;
    this->leftPID->SetI(I, PIDSlot::Velocity);
    this->rightPID->SetI(I, PIDSlot::Velocity);
}

void CANSparkMax_Drivetrain::SetD_velocity(double D){
    this->kD_velocity_ = D;
    this->leftPID->SetD(D, PIDSlot::Velocity);
    this->rightPID->SetD(D, PIDSlot::Velocity);
}

void CANSparkMax_Drivetrain::SetFF_velocity(double FF){
    this->kFF_velocity_ = FF;
    this->leftPID->SetFF(FF, PIDSlot::Velocity);
    this->rightPID->SetFF(FF, PIDSlot::Velocity);
}



void CANSparkMax_Drivetrain::SetP_position(double P){
    this->kP_position_ = P;
    this->leftPID->SetP(P, PIDSlot::Position);
    this->rightPID->SetP(P, PIDSlot::Position);
}

void CANSparkMax_Drivetrain::SetI_position(double I){
    this->kI_position_ = I;
    this->leftPID->SetI(I, PIDSlot::Position);
    this->rightPID->SetI(I, PIDSlot::Position);
}

void CANSparkMax_Drivetrain::SetD_position(double D){
    this->kD_position_ = D;
    this->leftPID->SetD(D, PIDSlot::Position);
    this->rightPID->SetD(D, PIDSlot::Position);
}

void CANSparkMax_Drivetrain::SetFF_position(double FF){
    this->kFF_position_ = FF;
    this->leftPID->SetFF(FF, PIDSlot::Position);
    this->rightPID->SetFF(FF, PIDSlot::Position);
}
