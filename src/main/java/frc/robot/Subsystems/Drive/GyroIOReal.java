// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;

/** Add your docs here. */
public class GyroIOReal implements GyroIO{
    Pigeon2 gyro;
    private StatusSignal<Angle> yaw;

    public GyroIOReal(int gyroID){
        gyro = new Pigeon2(gyroID);
        yaw = gyro.getYaw();
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(250);
        gyro.optimizeBusUtilization();
        gyro.reset();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        if (gyro.isConnected()) {
            inputs.isConnected = true;
        }
        inputs.yawHeading = gyro.getRotation2d();
    }

    @Override
    public void setGyro(){
        Rotation2d lockYaw = gyro.getRotation2d();
        gyro.setYaw(lockYaw.getRadians());
    }

    public double gyroAngle() {
        return gyro.getRotation2d().getRadians();
    }

    @Override
    public void reset() {
        gyro.reset();
    }

}
