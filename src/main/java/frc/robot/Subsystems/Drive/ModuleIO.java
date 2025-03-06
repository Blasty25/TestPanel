// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface ModuleIO {

    @AutoLog
    public static class ModuleIOInputs {
        public boolean runSysId = false;

        public double drivePosition = 0.0;
        public double driveCurrent = 0.0;
        public double driveVelocity = 0.0;
        public double driveFeedForward = 0.0;
        public double driveAppliedVolts = 0.0;

        public double turnPosition = 0.0;
        public double turnEncoder = 0.0;
        public double turnCurrent = 0.0;
        public double turnVelocity = 0.0;
        public double turnFeedforward = 0.0;
        public double turnAppliedVolts = 0.0;

        public double[] odometryTimestamps = new double[] {};
        public double[] odometryDrivePositionsRad = new double[] {};
        public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};

        public double[] driveVoltage = new double[] {};
    }

    public default void updateInputs(ModuleIOInputs inputs) {
    }

    public default void setDriveMotor(double positionRad, double feedForward) {
    }

    public default void setTurnMotor(double rotation) {
    }

    public default void resetMotors(double position) {
    }

    public default void getHeading(double heading) {
    }

    public default void setBrakeMode(boolean enabled) {
    }

    public default void runTurnPosition(Rotation2d rotation) {
    }

    public default void runCharacterization(double volts){

    }

}
