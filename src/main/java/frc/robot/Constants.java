// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/** Add your docs here. */
public class Constants {
    public static class DriveConstants {
        public static final double LOOP_UPDATE = 0.02;
        public static final DCMotor motor = DCMotor.getKrakenX60(1);

        public static final double driveGearing = 5.14;
        public static final double driveMOI = 0.025;

        public static final double turnGearing = 12.8; 
        public static final double turnMOI = 0.005;

        public static final double drivekP = 0.103;
        public static final double drivekI = 0.0;
        public static final double drivekD = 0.0;

        public static final double turnkP = 1.5;
        public static final double turnkI = 0.0;
        public static final double turnkD = 0.0;

        public static final double driveS = 0.0;
        public static final double driveV = 0.0;
        public static final double driveA = 0.0;

        public static final Distance wheelRadius = Inches.of(1.931);
        public static final double trackWidth = Units.inchesToMeters(10.0);
        public static final double negTrack = Units.inchesToMeters(10.0);
        public static final double posTrack = Units.inchesToMeters(10.0);

        public static final int driveCurrentLimitAmps = 40;
        public static final int turnCurrentLimitAmps = 40;

        public static final double drivePositionConversionFactor = 2 * Math.PI * wheelRadius.in(Meter)
                / driveGearing;
        public static final double turnPositionConversionFactor = 2 * Math.PI / turnGearing;

        public static final double driveVelocityFactor = drivePositionConversionFactor / 60.0;
        public static final double turnVelocityFactor = turnPositionConversionFactor / 60.0;

        public static final Translation2d[] moduletranslations = {
                new Translation2d(trackWidth, trackWidth), // FL
                new Translation2d(trackWidth, -trackWidth), // FR
                new Translation2d(-trackWidth, trackWidth), // BL
                new Translation2d(-trackWidth, -trackWidth) // BR
        };

        public static final double maxDriveSpeed = 4.75; // Meters per second
        public static final double maxAngularspeed = 3.75; // Figure out max speeds later

        public enum BOT {
            Protolone,
            Comp
        }

        public static final BOT type = BOT.Protolone;
        /// AUTOS
        public static RobotConfig config;
        public static final boolean tuningMode = true;
        public static final boolean alliance = false; // true : Red, false : Blue
        public static final CommandXboxController controller = new CommandXboxController(0);
    }
}
