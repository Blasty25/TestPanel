// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import java.util.Queue;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Subsystems.drive.util.SparkOdometryThread;

/** Add your docs here. */
public class GyroIOReal implements GyroIO {
    private final Pigeon2 gyro = new Pigeon2(9, "rio");
    private final StatusSignal<Angle> yaw = gyro.getYaw();
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    private final StatusSignal<AngularVelocity> yawVelocity = gyro.getAngularVelocityZWorld();

    public GyroIOReal(int gyroID) {
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(100.0);
        yawVelocity.setUpdateFrequency(50.0);
        gyro.optimizeBusUtilization();
        yawTimestampQueue = SparkOdometryThread.getInstance().makeTimestampQueue();
        yawPositionQueue = SparkOdometryThread.getInstance().registerSignal(yaw::getValueAsDouble);
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        if (gyro.isConnected()) {
            inputs.isConnected = true;
        }
        inputs.yawHeading = gyro.getRotation2d();
        inputs.odometryYawPositions = yawPositionQueue.stream()
                .map((Double value) -> Rotation2d.fromDegrees(value))
                .toArray(Rotation2d[]::new);
        inputs.odometryYawTimestamps = yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }

    @Override
    public void setGyro() {
        Rotation2d lockYaw = gyro.getRotation2d();
        gyro.setYaw(90);
    }

    public double gyroAngle() {
        return gyro.getRotation2d().getRadians();
    }

    public void reset() {
        gyro.setYaw(0);
    }

}
