/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "commands/TankDrive.h"



TankDrive::TankDrive(DrivetrainType* drivetrain, frc::XboxController* contoller) {
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements({drivetrain});
  this->drivetrain = drivetrain;
  this->controller = controller;
}

// Called when the command is initially scheduled.
void TankDrive::Initialize() {
  drivetrain->controlMode = Drivetrain::ControlMode::PERCENT;
}

// Called repeatedly when this Command is scheduled to run
void TankDrive::Execute() {
  drivetrain->leftTarget = controller->GetY(frc::GenericHID::kLeftHand);
  drivetrain->rightTarget = controller->GetY(frc::GenericHID::kRightHand);
}

// Called once the command ends or is interrupted.
void TankDrive::End(bool interrupted) {}

// Returns true when the command should end.
bool TankDrive::IsFinished() { return false; }
