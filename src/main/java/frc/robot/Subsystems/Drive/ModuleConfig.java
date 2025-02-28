// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

public class ModuleConfig {

    public Config configure(int index){
        Config config = null;
        switch (index) {
            case 0:
                config = new Config(1, 2, 0, 0.671, false);
                break;
            case 1:
                config = new Config(3, 4, 1, 0.186, false); 
                break;
            case 2: 
                config = new Config(5, 6, 2, 0.684, false);
                break;
            case 3:
                config = new Config(7, 8, 3, 0.005, false);
                break;
        }
        return config;
    }

}
