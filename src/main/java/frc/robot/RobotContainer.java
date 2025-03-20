// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Subsystems.Carriage.CarriageIOSim;
import frc.robot.Subsystems.Carriage.CarriageIOSparkMax;
import frc.robot.Subsystems.Carriage.CarriageSubsystem;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Subsystems.drive.GyroIO;
import frc.robot.Subsystems.drive.GyroIOPigeon2;
import frc.robot.Subsystems.drive.ModuleIOSim;
import frc.robot.Subsystems.drive.ModuleIOTalonFX;
import frc.robot.Subsystems.drive.TunerConstants;
import frc.robot.Subsystems.drive.Commands.DriveCommands;
import frc.robot.Subsystems.elevator.Elevator;
import frc.robot.Subsystems.elevator.Elevator.ElevatorSetpoint;
import frc.robot.Subsystems.elevator.ElevatorIOSim;
import frc.robot.Subsystems.elevator.ElevatorIOSpark;

public class RobotContainer {
  private Drive drive;
  private CarriageSubsystem carriage;
  private final Elevator elevator;
  private CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    if (Robot.isReal()) {
      drive = new Drive(
          new GyroIOPigeon2(),
          new ModuleIOTalonFX(TunerConstants.FrontLeft),
          new ModuleIOTalonFX(TunerConstants.FrontRight),
          new ModuleIOTalonFX(TunerConstants.BackLeft),
          new ModuleIOTalonFX(TunerConstants.BackRight));
      elevator = new Elevator(new ElevatorIOSpark());
      carriage = new CarriageSubsystem(new CarriageIOSparkMax());

    } else {
      drive = new Drive(
          new GyroIO() {
          },
          new ModuleIOSim(TunerConstants.FrontLeft),
          new ModuleIOSim(TunerConstants.FrontRight),
          new ModuleIOSim(TunerConstants.BackLeft),
          new ModuleIOSim(TunerConstants.BackRight));
      elevator = new Elevator(new ElevatorIOSim());
      carriage = new CarriageSubsystem(new CarriageIOSim());
    }

    configureBindings();
  }

  private void configureBindings() {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> controller.getLeftY(),
            () -> controller.getLeftX(),
            () -> -controller.getRightX(),
            () -> 0.1,//op ded
            () -> 1));
    controller
        .x()
        .whileTrue(
            DriveCommands.joystickDrive(
                drive,
                () -> -controller.getLeftY(),
                () -> -controller.getLeftX(),
                () -> -controller.getRightX(),
                () -> 0.1, //Op ded
                () -> 0.5));

    carriage.setDefaultCommand(carriage.setVolts(() -> controller.getRightTriggerAxis()));
    carriage.setDefaultCommand(carriage.setVolts(() -> -controller.getLeftTriggerAxis()));

    controller.y().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.L4));
    controller.x().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.L3));
    controller.b().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.L2));
    controller.a().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.L1));
    controller.povDown().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.ZERO));
    controller.povUp().onTrue(elevator.setSetpoint(() -> ElevatorSetpoint.INTAKE));

    // FYI IF USING SYS ID GO TO MODULEIO AND CHANGE RUNSYSID TO TRUE
    // controller.a().whileTrue(elevator.sysIdRoutine());
    // controller.b().whileTrue(drive.sysIDSwerve());
    // controller.a().whileTrue(elevator.sysIdRoutine());
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("path");
  }
}
