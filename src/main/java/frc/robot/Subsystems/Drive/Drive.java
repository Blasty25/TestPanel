// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class Drive extends SubsystemBase {
    private final Module[] modules = new Module[4];
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged moduleInputs = new ModuleIOInputsAutoLogged();

    private final GyroIOReal gyroReal = new GyroIOReal(9);

    static final Lock odeometryLock = new ReentrantLock();

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.moduletranslations);

    public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;

        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 3);
        modules[3] = new Module(brModuleIO, 3);

        // Set the Brake mode to each Module
        for (Module module : modules) {
            module.setBrakeMode(true);
        }
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        for (Module module : modules) {
            module.updateInputs();
        }

    }

    public void fieldRelative(ChassisSpeeds swerve) {
        setModule(ChassisSpeeds.fromFieldRelativeSpeeds(swerve, gyroInputs.yawHeading));
    }

    public void setModule(ChassisSpeeds moduleSpeed) {
        SwerveModuleState[] setpointUnoptimized = (kinematics.toSwerveModuleStates(moduleSpeed));
        Logger.recordOutput("Drive/Modules/SetpointUnoptimized", setpointUnoptimized);
        for(int i = 0; i < 4; i++){
            modules[i].setState(setpointUnoptimized[i]);
        }   
    }


    @AutoLogOutput(key = "Drive/Drivetrain/RobotPose")
    public Pose2d getRobotPose(SwerveDrivePoseEstimator currentState) {
        Pose2d robotPose = currentState.getEstimatedPosition();
        Logger.recordOutput("Drive/Drivetrain/RobotPose", robotPose);
        Logger.recordOutput("Drive/Modules/Setpoints", robotPose);
        return robotPose;
    }
    
    @AutoLogOutput(key = "Drive/Modules/SetpointsOptimized")
    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4]; 
        for(int i = 0; i < 4; i++){
            states[i] = modules[i].getState();
        }
        Logger.recordOutput("Drive/Modules/SetpointsOptimized", states);
        return states;
    }

    public Command drive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier) {
        return new RunCommand(
                () -> {
                    double speed = xSupplier.getAsDouble();
                    double strafe = ySupplier.getAsDouble();
                    double rotation = zSupplier.getAsDouble();

                    MathUtil.applyDeadband(speed, 0.2);
                    MathUtil.applyDeadband(strafe, 0.2);
                    MathUtil.applyDeadband(rotation, 0.2);

                    ChassisSpeeds newSpeeds = new ChassisSpeeds(speed, strafe, rotation);

                    this.fieldRelative(newSpeeds);

                }, this);
    }

}
