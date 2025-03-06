package frc.robot.Subsystems.Elevator;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.measure.Distance;

public class ElevatorConstants {
    public static final DCMotor motor = DCMotor.getNEO(2);

    public static final double mass = Units.lbsToKilograms(25.1); // change to actual weight
    public static final double radius = 5.0 / 1000.0 * 36 / (2.0 * Math.PI);
    public static final double gearing = (5.0 / 1.0);

    public static final double maxHeight = Units.inchesToMeters(44);
    public static final double minHeight = Units.inchesToMeters(0.0);
    public static final double startingHeight = minHeight;

    // sim

    // public static final double p = 23.918;
    // public static final double i = 0.0;
    // public static final double d = 0.45168;

    // public static final double s = 0.0;
    // public static final double g = 1.0612;
    // public static final double v = 5.87;
    // public static final double a = 0.12;

    public static final double p = 0.0;
    public static final double i = 0.0;
    public static final double d = 0.0;

    public static final double s = 0.27117;
    public static final double g = 0.42059;
    public static final double v = 6.049;
    public static final double a = 0.75;

    public static final double maxProfileVelocity = 0.5;
    public static final double maxProfileAcceleration = 0.5;

    public static final double maxProfileVoltage = 6.0;

    public static final int leftMotorId = 10;
    public static final int rightMotorId = 11;

    public static final double positionConversionFactor = radius * 2 * Math.PI / gearing;
    public static final double velocityConversionFactor = positionConversionFactor / 60.0;

    public static final Distance sysIdMinPosition = Distance.ofBaseUnits(0.1, Meters);
    public static final Distance sysIdMaxPosition = Distance.ofBaseUnits(1.5, Meters);

    public static final double sysIdRampUp = 2.5;
    public static final double sysIdStep = 5.5;
    public static final double sysIdTimeout = 20.0;
}