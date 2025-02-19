// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Carriage.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.Carriage.CarriageSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RunIntake extends Command {
  private CarriageSubsystem carriage;
  private double RPM;
  /** Creates a new RunCarriage. */
  public RunIntake(double RPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    carriage = CarriageSubsystem.getInstance();
    this.RPM = RPM;
    addRequirements(carriage);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    carriage.setRPM(RPM);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    carriage.settoZero();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;  
  }
}
