/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "subsystems/TalonSRX_Drivetrain.h"

TalonSRX_Drivetrain::TalonSRX_Drivetrain(int leftMotorCount, int rightMotorCount, bool invertForward, bool areSlavesVictorSPX){
    //error checking
    if(leftMotorCount == 0){
        throw "Left motor count must be greater than 0.";
        return;
    }
    
    if(rightMotorCount == 0){
        throw "Right motor count must be greater than 0.";
        return;
    }


    leftMaster = new WPI_TalonSRX(1);//ID 1
    rightMaster = new WPI_TalonSRX(4);//ID 4
 
    for(int i = 0; i < leftMotorCount - 1; ++i){//IDs range from 2 to 3
        if(areSlavesVictorSPX){
            leftSlaves[i] = new WPI_VictorSPX(i + 2);//creates the other motors as VictorSPXs if the slaves are VictorSPXs
        }else
            leftSlaves[i] = new WPI_TalonSRX(i + 2);

        leftSlaves[i]->Follow(*leftMaster);
    }

    for(int i = 0; i < rightMotorCount - 1; ++i){//IDs range from 5 to 6
        if(areSlavesVictorSPX){
            rightSlaves[i] = new WPI_VictorSPX(i + 5);//creates the other motors as VictorSPXs if the slaves are VictorSPXs
        }else
            rightSlaves[i] = new WPI_TalonSRX(i + 5);

        rightSlaves[i]->Follow(*rightMaster);
    }

    leftMaster->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative);
    rightMaster->ConfigSelectedFeedbackSensor(ctre::phoenix::motorcontrol::FeedbackDevice::CTRE_MagEncoder_Relative);

    //sets the forward direction
    leftMaster->SetInverted(!invertForward);//makes the left move forward
    rightMaster->SetInverted(invertForward);

    leftMaster->SetSensorPhase(!invertForward);//makes the left encoder read positive in the forward direction
    rightMaster->SetSensorPhase(invertForward);
}

void TalonSRX_Drivetrain::Periodic() {
    switch(controlMode){
        case Drivetrain::ControlMode::PERCENT:
            leftMaster->Set(leftTarget);
            rightMaster->Set(rightTarget);
            break;
        case Drivetrain::ControlMode::VELOCITY:
            //selects the proper PID values
            leftMaster->SelectProfileSlot(Drivetrain::PIDSlot::Velocity, 0);
            rightMaster->SelectProfileSlot(Drivetrain::PIDSlot::Velocity, 0);

            //Updates the PID target
            leftMaster->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, leftTarget);
            rightMaster->Set(ctre::phoenix::motorcontrol::ControlMode::Velocity, rightTarget);
            break;
        case Drivetrain::ControlMode::POSITION:
            //selects the proper PID values
            leftMaster->SelectProfileSlot(Drivetrain::PIDSlot::Position, 0);
            rightMaster->SelectProfileSlot(Drivetrain::PIDSlot::Position, 0);
            
            //Updates the PID target
            leftMaster->Set(ctre::phoenix::motorcontrol::ControlMode::Position, leftTarget);
            rightMaster->Set(ctre::phoenix::motorcontrol::ControlMode::Position, rightTarget);
            break;
    }
}



void TalonSRX_Drivetrain::SetP_velocity(double P){
    this->kP_velocity_ = P;
    this->leftMaster->Config_kP(TalonSRX_Drivetrain::PIDSlot::Velocity, P);
    this->rightMaster->Config_kP(TalonSRX_Drivetrain::PIDSlot::Velocity, P);
}

void TalonSRX_Drivetrain::SetI_velocity(double I){
    this->kI_velocity_ = I;
    this->leftMaster->Config_kI(TalonSRX_Drivetrain::PIDSlot::Velocity, I);
    this->rightMaster->Config_kI(TalonSRX_Drivetrain::PIDSlot::Velocity, I);
}

void TalonSRX_Drivetrain::SetD_velocity(double D){
    this->kD_velocity_ = D;
    this->leftMaster->Config_kD(TalonSRX_Drivetrain::PIDSlot::Velocity, D);
    this->rightMaster->Config_kD(TalonSRX_Drivetrain::PIDSlot::Velocity, D);
}

void TalonSRX_Drivetrain::SetFF_velocity(double FF){
    this->kFF_velocity_ = FF;
    this->leftMaster->Config_kF(TalonSRX_Drivetrain::PIDSlot::Velocity, FF);
    this->rightMaster->Config_kF(TalonSRX_Drivetrain::PIDSlot::Velocity, FF);
}



void TalonSRX_Drivetrain::SetP_position(double P){
    this->kP_position_ = P;
    this->leftMaster->Config_kP(TalonSRX_Drivetrain::PIDSlot::Position, P);
    this->rightMaster->Config_kP(TalonSRX_Drivetrain::PIDSlot::Position, P);
}

void TalonSRX_Drivetrain::SetI_position(double I){
    this->kI_position_ = I;
    this->leftMaster->Config_kI(TalonSRX_Drivetrain::PIDSlot::Position, I);
    this->rightMaster->Config_kI(TalonSRX_Drivetrain::PIDSlot::Position, I);
}

void TalonSRX_Drivetrain::SetD_position(double D){
    this->kD_position_ = D;
    this->leftMaster->Config_kD(TalonSRX_Drivetrain::PIDSlot::Position, D);
    this->rightMaster->Config_kD(TalonSRX_Drivetrain::PIDSlot::Position, D);
}

void TalonSRX_Drivetrain::SetFF_position(double FF){
    this->kFF_position_ = FF;
    this->leftMaster->Config_kF(TalonSRX_Drivetrain::PIDSlot::Position, FF);
    this->rightMaster->Config_kF(TalonSRX_Drivetrain::PIDSlot::Position, FF);
}