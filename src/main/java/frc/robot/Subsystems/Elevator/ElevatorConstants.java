// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;


import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorConstants {
    public static final double kS = 0.0;
    public static final double kG = 0.0;
    public static final double kA = 0.0;
    public static final double kV = 0.0;

    public static final DCMotor elevatorMotor = DCMotor.getNEO(1);
    public static final double elevatorMass = Units.lbsToKilograms(30.0);
    public static final double elevatorRadius = Units.inchesToMeters(0.9175);
    public static final double gearing = 6.75;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;

    public static final double positionConversionFactor = 2.0 * Math.PI * elevatorRadius / gearing;
    public static final double velocityConversionFactor = positionConversionFactor / 60.0;
}
