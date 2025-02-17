// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.Drive.Drive;
import frc.robot.Subsystems.Drive.GyroIO;
import frc.robot.Subsystems.Drive.GyroIOReal;
import frc.robot.Subsystems.Drive.Module;
import frc.robot.Subsystems.Drive.ModuleConfig;
import frc.robot.Subsystems.Drive.Config;
import frc.robot.Subsystems.Drive.ModuleIO;
import frc.robot.Subsystems.Drive.ModuleIOReal;
import frc.robot.Subsystems.Drive.ModuleIOSim;

public class RobotContainer {
  private Drive drive;
  private CommandXboxController controller = new CommandXboxController(0);

  public RobotContainer() {
    if (Robot.isReal()) {
      drive = new Drive(
        new GyroIOReal(9),
        new ModuleIOReal(new ModuleConfig().configure(0)),
        new ModuleIOReal(new ModuleConfig().configure(1)),
        new ModuleIOReal(new ModuleConfig().configure(2)),
        new ModuleIOReal(new ModuleConfig().configure(3))
      );
    }else{
      drive = new Drive(
        new GyroIO() {},
        new ModuleIOSim(),
        new ModuleIOSim(),
        new ModuleIOSim(),
        new ModuleIOSim()
      );
    }

    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
      drive.drive(
        ()-> -controller.getLeftY(),
        ()-> -controller.getLeftX(),
        ()-> -controller.getRightX()
      )
    );
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
