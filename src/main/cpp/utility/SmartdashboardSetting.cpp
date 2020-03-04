/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#include "utility/SmartdashboardSetting.h"

SmartdashboardSetting::SmartdashboardSetting() {}

void SmartdashboardSetting::AddOption(std::string name){
    options.push_back(name);
    ++numberOfOptions;
    chooser.AddOption(name, numberOfOptions);
}

void SmartdashboardSetting::SetDefaultOption(std::string name){
    options.push_back(name);
    ++numberOfOptions;
    chooser.SetDefaultOption(name, numberOfOptions);
}

int SmartdashboardSetting::GetSelected(){
    return chooser.GetSelected();
}

bool SmartdashboardSetting::IsSelected(std::string name){
    return FindSettingId(name) == GetSelected();
}

int SmartdashboardSetting::FindSettingId(std::string name){

    for(int i = 0; i < options.size(); ++i){
        if(options[i] == name){
            return i + 1;
        }
    }

    return 0;
}