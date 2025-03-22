// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

public class ModuleConfig {

    public Config configure(int index){
        Config config;
        switch (index) {
            case 0:
                config = new Config(1, 2, 0, 0.06, true);
                return config;
            case 1:
                config = new Config(3, 4, 1, 0.73, true);
                return config;
            case 2: 
                config = new Config(5, 6, 2, 0.06, true);
                return config;
            case 3:
                config = new Config(7, 8, 3, 0.49, true);
                return config;
            default:
                config = new Config(0, 0, 5, 0.0, false); 
                return config;
        }
    }

}
