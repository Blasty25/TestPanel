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
import frc.robot.Subsystems.drive.pathfinding.leftPIDAllign;
import frc.robot.Subsystems.drive.pathfinding.rightPIDAllign;

public class RobotContainer {
  private Drive drive;
  private Elevator elevator;

  public RobotContainer() {
    if (Robot.isReal()) {
      drive = new Drive(
          new GyroIOReal(9),
          new ModuleIOSparkMax(new ModuleConfig().configure(0)),
          new ModuleIOSparkMax(new ModuleConfig().configure(1)),
          new ModuleIOSparkMax(new ModuleConfig().configure(2)),
          new ModuleIOSparkMax(new ModuleConfig().configure(3)));
      elevator = new Elevator(new ElevatorIOReal());

    } else {
      drive = new Drive(
          new GyroIO() {
          },
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
           () -> 0.2,  //deadband
           () -> 1));  //max speed 

    // Reseting Gyro and Locking Gyro features
    // DriveConstants.controller.a().onTrue(new lockGyro(pigeon));
    // DriveConstants.controller.b().onTrue(drive.resetGyro());
    DriveConstants.controller.leftBumper().onTrue(new leftPIDAllign(drive, DriveConstants.alliance));
    DriveConstants.controller.rightBumper().onTrue(new rightPIDAllign(drive, DriveConstants.alliance));
    // DriveConstants.controller.y().whileTrue(drive.recordPose());

    DriveConstants.controller.y().onTrue(elevator.runSetpoint(ElevatorConstants.l4));
    DriveConstants.controller.x().onTrue(elevator.runSetpoint(ElevatorConstants.l3));
    DriveConstants.controller.b().onTrue(elevator.runSetpoint(ElevatorConstants.l2));
    DriveConstants.controller.a().onTrue(elevator.runSetpoint(ElevatorConstants.l1));
    DriveConstants.controller.povDown().onTrue(elevator.runSetpoint(ElevatorConstants.stow));

    DriveConstants.controller.povUp().onTrue(elevator.resetEncoder());

    // FYI IF USING SYS ID GO TO MODULEIO AND CHANGE RUNSYSID TO TRUE
    // DriveConstants.controller.a().whileTrue(elevator.sysIdRoutine());
    // controller.b().whileTrue(drive.sysIDSwerve());
    // controller.a().whileTrue(elevator.sysIdRoutine());
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("middle");
  }
}
