// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.annotation.ElementType;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.CommandUtil.runReef;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Carriage.CarriageConstants;
import frc.robot.Subsystems.Carriage.CarriageIOSim;
import frc.robot.Subsystems.Carriage.CarriageIOSparkMax;
import frc.robot.Subsystems.Carriage.CarriageIOTalonSRX;
import frc.robot.Subsystems.Carriage.CarriageSubsystem;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.drive.GyroIO;
import frc.robot.Subsystems.drive.GyroIOReal;
import frc.robot.Subsystems.drive.ModuleConfig;
import frc.robot.Subsystems.drive.util.lockGyro;
import frc.robot.Subsystems.drive.ModuleIOSim;
import frc.robot.Subsystems.drive.ModuleIOSparkMax;
import frc.robot.Subsystems.drive.pathfinding.PIDAlign;
import frc.robot.Subsystems.elevator.Elevator;
import frc.robot.Subsystems.elevator.Elevator.ElevatorSetpoint;
import frc.robot.Subsystems.elevator.ElevatorIOSim;
import frc.robot.Subsystems.elevator.ElevatorIOSpark;

public class RobotContainer {
  private Drive drive;
  private Elevator elevator;
  private CarriageSubsystem carriage;
  private GyroIOReal pigeon;

  public RobotContainer() {
    if (Robot.isReal()) {
          drive = new Drive(
              new GyroIOReal(9),
              new ModuleIOSparkMax(new ModuleConfig().configure(0)),
              new ModuleIOSparkMax(new ModuleConfig().configure(1)),
              new ModuleIOSparkMax(new ModuleConfig().configure(2)),
              new ModuleIOSparkMax(new ModuleConfig().configure(3)));
              elevator = new Elevator(new ElevatorIOSpark());
              // carriage = new CarriageSubsystem(new CarriageIOTalonSRX());

    } else {
      drive = new Drive(
          new GyroIO() {
          },
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim(),
          new ModuleIOSim());
      elevator = new Elevator(new ElevatorIOSim());
      carriage = new CarriageSubsystem(new CarriageIOSim());
    }

    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        drive.joystickDrive(
            () -> Constants.DriveConstants.controller.getLeftY(),
            () -> Constants.DriveConstants.controller.getLeftX(),
            () -> Constants.DriveConstants.controller.getRightX(),
            () -> 0.1,
            () -> 1));

    // carriage.setDefaultCommand(carriage.setVolts(()-> controller.getRightTriggerAxis()));
    // carriage.setDefaultCommand(carriage.setVolts(()-> -controller.getLeftTriggerAxis()));


    DriveConstants.controller.y().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.L4));
    DriveConstants.controller.x().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.L3));
    DriveConstants.controller.b().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.L2));
    DriveConstants.controller.a().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.L1));
    DriveConstants.controller.povDown().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.ZERO));
    DriveConstants.controller.povUp().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.INTAKE));

    // Reseting Gyro and Locking Gyro features
    DriveConstants.controller.a().onTrue(new lockGyro(pigeon));
    DriveConstants.controller.b().onTrue(drive.resetGyro());
    DriveConstants.controller.leftBumper().onTrue(new PIDAlign(drive));
    DriveConstants.controller.y().whileTrue(drive.recordPose());

    // FYI IF USING SYS ID GO TO MODULEIO AND CHANGE RUNSYSID TO TRUE
    // controller.a().whileTrue(elevator.sysIdRoutine());
    // controller.b().whileTrue(drive.sysIDSwerve());
    // controller.a().whileTrue(elevator.sysIdRoutine());
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("middle");
  }
}
