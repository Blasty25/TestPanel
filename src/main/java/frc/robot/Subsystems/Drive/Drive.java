// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class Drive extends SubsystemBase {
    private final Module[] modules = new Module[4];
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();


    static final Lock odeometryLock = new ReentrantLock();

    private Rotation2d gyroEstimator = new Rotation2d();
    private SwerveModulePosition[] firstPositions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
    };

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.moduletranslations);
    private final SwerveDrivePoseEstimator newPose = new SwerveDrivePoseEstimator(kinematics,new Rotation2d(), firstPositions, new Pose2d());
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, gyroInputs.yawHeading, firstPositions);

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

        SwerveModulePosition[] modulePositions = modulePositions();

        SwerveModulePosition[] finalModules = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++){
            finalModules[i] = new SwerveModulePosition(
            modulePositions[i].distanceMeters - firstPositions[i].distanceMeters,
            modulePositions[i].angle
            );
        }

        Twist2d twist = kinematics.toTwist2d(finalModules);
        gyroEstimator = gyroEstimator.plus(new Rotation2d(twist.dtheta));
        firstPositions = finalModules;

        gyroInputs.RobotPose = odometry.update(gyroEstimator, modulePositions);
        

        // if (Robot.isSimulation()) {
        //     SwerveModulePosition[] wheelPositions = new SwerveModulePosition[4];
        //     for(int i = 0; i < 4; i++){
        //         wheelPositions[i] = new SwerveModulePosition(
        //         modulePositions[i].distanceMeters - firstPositions[i].distanceMeters,
        //         modulePositions[i].angle
        //         );
        //     }
        //     Twist2d twist = kinematics.toTwist2d(wheelPositions);
        //     gyroEstimator = gyroEstimator.plus(new Rotation2d(twist.dtheta));
        //     firstPositions = wheelPositions;
        // }
        // newPose.update(gyroEstimator, modulePositions);

    }

    public SwerveModulePosition[] modulePositions(){
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++){
            modulePositions[i] = modules[i].getPosition();
        }
        return modulePositions;
    }

    // public void fieldRelative(ChassisSpeeds swerve) {
    //     setModule(ChassisSpeeds.fromFieldRelativeSpeeds(swerve, gyroInputs.yawHeading));
    // }



    // public void setModule(ChassisSpeeds moduleSpeed) {
    //     SwerveModuleState[] setpointUnoptimized = (kinematics.toSwerveModuleStates(moduleSpeed));
    //     Logger.recordOutput("Drive/Modules/SetpointUnoptimized", setpointUnoptimized);
    //     for(int i = 0; i < 4; i++){
    //         modules[i].setState(setpointUnoptimized[i]);
    //     }   
    // }


    @AutoLogOutput(key = "Drive/Drivetrain/RobotPose")
    public Pose2d getRobotPose() {
        return newPose.getEstimatedPosition();
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

    // public Command drive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier) {
    //     return new RunCommand(
    //             () -> {
    //                 double speed = xSupplier.getAsDouble();
    //                 double strafe = ySupplier.getAsDouble();
    //                 double rotation = zSupplier.getAsDouble();

    //                 MathUtil.applyDeadband(speed, 0.1);
    //                 MathUtil.applyDeadband(strafe, 0.15);
    //                 MathUtil.applyDeadband(rotation, 0.1);

    //                 double xVal = speed * DriveConstants.maxDriveSpeed;
    //                 double yVal = strafe * DriveConstants.maxDriveSpeed;
    //                 double rot = rotation * DriveConstants.maxAngularspeed;
                    

    //                 ChassisSpeeds newSpeeds = new ChassisSpeeds(xVal, yVal, rot);

    //                 this.fieldRelative(newSpeeds);

    //             }, this);
    // }

    public Command fieldOriantedDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier) {
        return new RunCommand(
                () -> {
                SwerveModuleState[] swervemodulestate = kinematics.toSwerveModuleStates(
                    ChassisSpeeds.discretize(
                                ChassisSpeeds.fromFieldRelativeSpeeds(
                                    (MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.1))* DriveConstants.maxDriveSpeed ,(MathUtil.applyDeadband((ySupplier.getAsDouble()), 0.1)) * DriveConstants.maxDriveSpeed,
                                    (MathUtil.applyDeadband(zSupplier.getAsDouble(),0.1)) * DriveConstants.maxAngularspeed ,gyroInputs.yawHeading) ,0.02));

                        for(int i = 0; i < 4; i++){
                            modules[i].setState(swervemodulestate[i]);
                        }
                        // Logger.recordOutput("Drive/Modules/SetpointsOptimized", swervemodulestate);
                }, this);
    }

}
