// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.elevator.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.elevator.Elevator;
import frc.robot.Subsystems.elevator.Elevator.ElevatorSetpoint;
import org.littletonrobotics.junction.Logger;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Reset extends Command {
  public Elevator elevator;

  public Reset(Elevator elevator) {
    this.elevator = elevator;
  }

  @Override
  public void initialize() {
    Logger.recordOutput("Elevator/Auto/Setpoint", elevator.getSetpoint());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setSetpoint(() -> ElevatorSetpoint.ZERO);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
