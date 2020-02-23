/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#define DrivetrainType TalonSRX_Drivetrain
#ifndef DrivetrainType
#error The type of drivetrain needs to be specified. \
To do this define DrivetrainType before this error.
#endif

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc/XboxController.h>

#include "subsystems/TalonSRX_Drivetrain.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class TankDrive
    : public frc2::CommandHelper<frc2::CommandBase, TankDrive> {
 public:
  //Uses a Drivetrain pointer so that classes that inherit Drivetrain can be used
  //Uses an XboxController pointer to lower runtime lag

  TankDrive(DrivetrainType*, frc::XboxController*);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

  DrivetrainType *drivetrain;
  frc::XboxController *controller;
};
