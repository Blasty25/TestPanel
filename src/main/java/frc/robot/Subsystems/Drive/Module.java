// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.DriveConstants;

public class Module {
    private final int index;
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();

    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(DriveConstants.driveS, DriveConstants.driveV, DriveConstants.driveA);


    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;

    }

    public void updateInputs() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + index, inputs);
    }

    // Returns the Angle of the Module
    public Rotation2d getAngle() {
        return inputs.turnPosition;
    }

    // Returns the drive position in Meters
    public double getPositionMeters() {
        return inputs.drivePosition * DriveConstants.wheelRadius;
    }

    public void setState(SwerveModuleState state){
      state.optimize(inputs.turnPosition);
      io.setDriveMotor(state.speedMetersPerSecond, driveFeedforward.calculate(state.speedMetersPerSecond));
      io.setTurnMotor(state.angle);
      inputs.driveVelocity = state.speedMetersPerSecond;
      inputs.turnPosition = state.angle;
    }

    public double getVelocityMetersPerSec() {
        return inputs.driveVelocity * DriveConstants.wheelRadius;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }


    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }

}
