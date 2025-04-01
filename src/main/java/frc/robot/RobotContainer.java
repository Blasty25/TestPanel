// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.drive.GyroIO;
import frc.robot.Subsystems.drive.GyroIOReal;
import frc.robot.Subsystems.drive.ModuleConfig;
import frc.robot.Subsystems.elevator.Elevator;
import frc.robot.Subsystems.elevator.ElevatorConstants;
import frc.robot.Subsystems.elevator.ElevatorIOReal;
import frc.robot.Subsystems.elevator.ElevatorIOSim;
import frc.robot.Subsystems.drive.ModuleIOSim;
import frc.robot.Subsystems.drive.ModuleIOSparkMax;
import frc.robot.Subsystems.drive.commands.ResetGyro;
import frc.robot.Subsystems.drive.pathfinding.leftPIDAllign;
import frc.robot.Subsystems.drive.pathfinding.rightPIDAllign;
import frc.robot.Subsystems.vision.*;

import static frc.robot.Constants.DriveConstants.*;

public class RobotContainer {
  private Drive drive;
  private Elevator elevator;
  private GyroIOReal gyro;
  private Vision vision;

  public RobotContainer() {
    if (Robot.isReal()) {
      gyro = new GyroIOReal(9);
      vision = new Vision(
          new CameraIOReal(VisionConstants.lCameraName, VisionConstants.lCameraTransform),
          new CameraIOReal(VisionConstants.rCameraName,
              VisionConstants.rCameraTransform));
      drive = new Drive(
          gyro,
          vision,
          new ModuleIOSparkMax(new ModuleConfig().configure(0)),
          new ModuleIOSparkMax(new ModuleConfig().configure(1)),
          new ModuleIOSparkMax(new ModuleConfig().configure(2)),
          new ModuleIOSparkMax(new ModuleConfig().configure(3)));
      elevator = new Elevator(new ElevatorIOReal());

    } else {
      gyro = null;
      vision = new Vision(
          new CameraIOSim(VisionConstants.lCameraName, VisionConstants.lCameraTransform),
          new CameraIOSim(VisionConstants.rCameraName, VisionConstants.rCameraTransform));
      drive = new Drive(
          new GyroIO() {},
          vision,
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim());
      elevator = new Elevator(new ElevatorIOSim());
    }

    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        drive.joystickDrive(
            () -> -Constants.DriveConstants.controller.getLeftY(),
            () -> -Constants.DriveConstants.controller.getLeftX(),
            () -> -Constants.DriveConstants.controller.getRightX(),
            () -> 0.25, // deadband
            () -> 0.5)); // max speed

    // Reseting Gyro and Locking Gyro features
    controller.leftBumper().onTrue(new leftPIDAllign(drive, DriveConstants.alliance));
    controller.rightBumper().onTrue(new rightPIDAllign(drive, DriveConstants.alliance));
    // DriveConstants.controller.y().whileTrue(drive.recordPose());

    controller.y().onTrue(elevator.runSetpoint(ElevatorConstants.l4));
    controller.x().onTrue(elevator.runSetpoint(ElevatorConstants.l3));
    controller.b().onTrue(elevator.runSetpoint(ElevatorConstants.l2));
    controller.a().onTrue(elevator.runSetpoint(ElevatorConstants.l1));
    controller.povDown().onTrue(elevator.runSetpoint(ElevatorConstants.stow));

    controller.povUp().onTrue(elevator.resetEncoder());

    controller.povRight().onTrue(
        drive.resetGyro(
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX(),
            () -> 0.2,
            () -> 1));

    DriveConstants.controller.povLeft().onTrue(new ResetGyro(gyro));
    // FYI IF USING SYS ID GO TO MODULEIO AND CHANGE RUNSYSID TO TRUE
    // DriveConstants.controller.a().whileTrue(elevator.sysIdRoutine());
    // controller.b().whileTrue(drive.sysIDSwerve());
    // controller.a().whileTrue(elevator.sysIdRoutine());
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("middle");
  }
}
