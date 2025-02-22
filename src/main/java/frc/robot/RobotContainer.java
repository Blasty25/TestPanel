// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.CarriageConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Carriage.Commands.RunIntake;
import frc.robot.Subsystems.Carriage.Commands.RunOuttake;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.GyroIO;
import frc.robot.Subsystems.Drive.GyroIOReal;
import frc.robot.Subsystems.Drive.ModuleConfig;
import frc.robot.Subsystems.Drive.ModuleIOTalonFX;
import frc.robot.Subsystems.Drive.ModuleIOSim;
import frc.robot.Subsystems.Drive.ModuleIOSparkMax;


public class RobotContainer {
  private Drive drive;
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
          break;

        case Comp:
          drive = new Drive(
              new GyroIOReal(9),
              new ModuleIOTalonFX(new ModuleConfig().configure(0)),
              new ModuleIOTalonFX(new ModuleConfig().configure(1)),
              new ModuleIOTalonFX(new ModuleConfig().configure(2)),
              new ModuleIOTalonFX(new ModuleConfig().configure(3)));
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
    }

    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        drive.fieldOriantedDrive(
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    controller.b().whileTrue(new RunIntake(CarriageConstants.maxSpeed));
    // controller.y().whileTrue(new RunOuttake(CarriageConstants.maxSpeed));

    // controller.a().whileTrue(elevator.sysIdRoutine());
    // controller.b().whileTrue(drive.sysIDSwerve());
  }

  public Command getAutonomousCommand() {
    return new PathPlannerAuto("path");
  }
}
