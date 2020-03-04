/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
#include <vector>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/CommandHelper.h>

class SmartdashboardSetting {
 private:
  frc::SendableChooser<int> chooser;
  int numberOfOptions = 0;

 public:
  std::vector<std::string> options;

  SmartdashboardSetting();

  void AddOption(std::string name);
  void SetDefaultOption(std::string name);
  int GetSelected();
  bool IsSelected(std::string name);
  int FindSettingId(std::string name);
};
