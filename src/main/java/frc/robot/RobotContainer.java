// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CommandUtil.runReef;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Carriage.CarriageConstants;
import frc.robot.Subsystems.Carriage.CarriageIOSim;
import frc.robot.Subsystems.Carriage.CarriageIOSparkMax;
import frc.robot.Subsystems.Carriage.CarriageIOTalonSRX;
import frc.robot.Subsystems.Carriage.CarriageSubsystem;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.GyroIO;
import frc.robot.Subsystems.Drive.GyroIOReal;
import frc.robot.Subsystems.Drive.ModuleConfig;
import frc.robot.Subsystems.Drive.ModuleIOTalonFX;
import frc.robot.Subsystems.Drive.util.lockGyro;
import frc.robot.Subsystems.Drive.ModuleIOSim;
import frc.robot.Subsystems.Drive.ModuleIOSparkMax;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorConstants;
import frc.robot.Subsystems.Elevator.ElevatorIOReal;
import frc.robot.Subsystems.Elevator.ElevatorIOSim;
import frc.robot.Subsystems.Elevator.Commands.setPosition;

public class RobotContainer {
  private Drive drive;
  private Elevator elevator;
  private CarriageSubsystem carriage;
  private GyroIOReal pigeon;
  private CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    if (Robot.isReal()) {
      switch (DriveConstants.type) {
        case Protolone:
          drive = new Drive(
              new GyroIOReal(9),
              new ModuleIOSparkMax(new ModuleConfig().configure(0)),
              new ModuleIOSparkMax(new ModuleConfig().configure(1)),
              new ModuleIOSparkMax(new ModuleConfig().configure(2)),
              new ModuleIOSparkMax(new ModuleConfig().configure(3)));
          elevator = new Elevator(new ElevatorIOReal(ElevatorConstants.leftMotorId, ElevatorConstants.rightMotorId));
          carriage = new CarriageSubsystem(new CarriageIOTalonSRX());
          break;

        case Comp:
          drive = new Drive(
              new GyroIOReal(0),
              new ModuleIOTalonFX(new ModuleConfig().configure(0)),
              new ModuleIOTalonFX(new ModuleConfig().configure(1)),
              new ModuleIOTalonFX(new ModuleConfig().configure(2)),
              new ModuleIOTalonFX(new ModuleConfig().configure(3)));
          elevator = new Elevator(new ElevatorIOReal(ElevatorConstants.leftMotorId, ElevatorConstants.rightMotorId));
          carriage = new CarriageSubsystem(new CarriageIOSparkMax());
          break;
      }

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
        drive.fieldOriantedDrive(
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    carriage.setDefaultCommand(carriage.setVolts(()-> controller.getRightTriggerAxis()));
    carriage.setDefaultCommand(carriage.setVolts(()-> -controller.getLeftTriggerAxis()));


    controller.povLeft().onTrue(new setPosition(elevator, 0.4));
    controller.povUp().onTrue(new setPosition(elevator, 1.4));
    controller.povRight().onTrue(new setPosition(elevator, 0.6));
    controller.povDown().onTrue(new setPosition(elevator, 0.0));

    // Reseting Gyro and Locking Gyro features
    controller.a().whileTrue(new lockGyro(pigeon));
    controller.b().whileTrue(drive.resetGyro());

    controller.y().whileTrue(drive.recordPose());

    // FYI IF USING SYS ID GO TO MODULEIO AND CHANGE RUNSYSID TO TRUE
    // controller.a().whileTrue(elevator.sysIdRoutine());
    // controller.b().whileTrue(drive.sysIDSwerve());
    // controller.a().whileTrue(elevator.sysIdRoutine());
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("path");
  }
}
