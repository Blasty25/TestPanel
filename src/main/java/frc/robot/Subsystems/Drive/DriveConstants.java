// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;

/** Add your docs here. */
public class DriveConstants {
    public static final double LOOP_UPDATE = 0.02;
    public static final DCMotor motor = DCMotor.getKrakenX60(1);

    public static final double driveGearing = 6.746;
    public static final double driveMOI = 0.02;

    public static final double turnGearing = 150.0 / 7.0;
    public static final double turnMOI = 0.005;

    public static final double drivekP = 0.06; // 0.8
    public static final double drivekI = 0.0;
    public static final double drivekD = 0.0;

    public static final double turnkP = 0.15;
    public static final double turnkI = 0.0;
    public static final double turnkD = 0.0;

    public static final double driveS = 0.0;
    public static final double driveV = 0.0;
    public static final double driveA = 0.0;

    public static final double wheelRadius = Units.inchesToMeters(2.00);
    public static final double trackWidth = Units.inchesToMeters(16.0);

    public static final int driveCurrentLimitAmps = 80;
    public static final int turnCurrentLimitAmps = 40;

    public static final double drivePositionConversionFactor = 2.0 * Math.PI * wheelRadius / driveGearing;
    public static final double turnPositionConversionFactor = 2.0 * Math.PI / turnGearing;

    public static final double driveVelocityFactor = drivePositionConversionFactor / 60.0;
    public static final double turnVelocityFactor = turnPositionConversionFactor / 60.0;

    public static final Translation2d[] moduletranslations = {
            new Translation2d(trackWidth, trackWidth), // FL
            new Translation2d(trackWidth, -trackWidth), // FR
            new Translation2d(-trackWidth, trackWidth), // BL
            new Translation2d(-trackWidth, -trackWidth) // BR
    };

    public static final double maxDriveSpeed = 1; // Meters per second
    public static final double maxAngularspeed = 2; // Figure out max speeds later

    public enum BOT {
        Protolone,
        Comp
    }

    public static final BOT type = BOT.Protolone;
    /// AUTOS
    public static RobotConfig config;

    public static class CarriageConstants {
        public static final int carriageId = 20;
        public static final double maxSpeed = 1.0; // 0-1 1 MAX
    }
}
