// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.CommandUtil;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.Carriage.CarriageSubsystem;
import frc.robot.Subsystems.Carriage.Commands.RunIntake;
import frc.robot.Subsystems.Carriage.Commands.RunOuttake;
import frc.robot.Subsystems.Carriage.Commands.autoIntake;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.Commands.setPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
public class runReef extends SequentialCommandGroup {
  /** Creates a new runReef. */
  public Elevator elevator;
  public CarriageSubsystem carriage;
  public double height;
  public double speed;

  public runReef(Elevator elevator, CarriageSubsystem carriage, double height, double speed) {
    this.elevator = elevator;
    this.carriage = carriage;
    this.height = height;
    this.speed = speed;
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new setPosition(elevator, height), new RunOuttake(speed));
  }
}
