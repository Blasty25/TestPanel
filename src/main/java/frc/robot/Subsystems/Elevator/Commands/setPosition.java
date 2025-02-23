// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator.Commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.Elevator.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class setPosition extends InstantCommand {
  public Elevator elevator;
  public double height;
  public setPosition(Elevator elevator, double height) {
    this.elevator = elevator;
    this.height = height;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setPosition(height);
  }
}
