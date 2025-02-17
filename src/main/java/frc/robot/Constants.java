// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Array;
import java.security.PublicKey;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Subsystems.Drive.Config;

/** Add your docs here. */
public class Constants {
    public static class DriveConstants {

        public static final double LOOP_UPDATE = 0.02;
        public static final DCMotor motor = DCMotor.getKrakenX60(1);

        public static final double driveGearing = 5.36;
        public static final double driveMOI = 0.02;

        public static final double turnGearing = 150.0 / 7.0;
        public static final double turnMOI = 0.005;

        public static final double drivekP = 0.2;  //0.8
        public static final double drivekI = 0.1;
        public static final double drivekD = 0.1;

        public static final double turnkP = 0.2;
        public static final double turnkI = 0.1;
        public static final double turnkD = 0.1;

        public static final double driveS = 0.0;
        public static final double driveV = 0.1;
        public static final double driveA = 0.1;

        public static final double wheelRadius = Units.inchesToMeters(2.00);
        public static final double trackWidth = Units.inchesToMeters(10.0);

        private static final double driveCurrentLimitAmps = 80;
        private static final double turnCurrentLimitAmps = 40;

        public static final Translation2d[] moduletranslations = {
                new Translation2d(trackWidth, trackWidth),  //FL
                new Translation2d(trackWidth, -trackWidth), //FR
                new Translation2d(-trackWidth, trackWidth), //BL
                new Translation2d(-trackWidth, -trackWidth) //BR
        };
        

        public static final double maxDriveSpeed = 4.0; //Meters per second
        public static final double maxAngularspeed = 5; //Figure out max speeds later

    }

    
}
