// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs{
        public boolean isConnected = false;
        public double heading = 0.0;
        public Rotation2d yawHeading = new Rotation2d();
        public double gyroVoltage = 0.0;
    }

    public default void updateInputs(GyroIOInputs inputs) {}

    public default void reset() {}
}
