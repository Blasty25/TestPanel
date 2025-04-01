// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface GyroIO {
    @AutoLog
    public static class GyroIOInputs {
        public boolean isConnected = false;
        public Rotation2d yawHeading = new Rotation2d();
        public Rotation2d[] odometryYawPositions = new Rotation2d[] {};
        public double[] odometryYawTimestamps = new double[] {};
    }

    public default void updateInputs(GyroIOInputs inputs) {
    }

    public default void setGyro() {}

    public default double getHeading() {return 0.0;}

    public default void reset() {}
}
