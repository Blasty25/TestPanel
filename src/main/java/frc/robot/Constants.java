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

        public static final double drivekP = 0.0;
        public static final double drivekI = 0.0;
        public static final double drivekD = 0.0;

        public static final double turnkP = 0.0;
        public static final double turnkI = 0.0;
        public static final double turnkD = 0.0;

        public static final double driveS = 0.0;
        public static final double driveV = 0.0;
        public static final double driveA = 0.0;

        public static final double turnS = 0.0;
        public static final double turnV = 0.0;
        public static final double turnA = 0.0;

        public static final double wheelRadius = Units.inchesToMeters(2.00);
        public static final double trackWidth = Units.inchesToMeters(10.0);

        private static final double driveCurrentLimitAmps = 80;
        private static final double turnCurrentLimitAmps = 40;

        public static final Translation2d[] moduletranslations = {
                new Translation2d(trackWidth, trackWidth),
                new Translation2d(trackWidth, -trackWidth),
                new Translation2d(-trackWidth, trackWidth),
                new Translation2d(-trackWidth, -trackWidth)
        };
        
        public static final double trackWidthX = Units.inchesToMeters(20.75);
        public static final double trackWidthY = Units.inchesToMeters(20.75);
        public static final double driveBaseRadus = Math.hypot(trackWidthX / 2, trackWidthY /2);
        public static final double maxAngularspeed = 4.69 / driveBaseRadus;
    }

    
}
