// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Carriage;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CarriageSubsystem extends SubsystemBase {
  
  private CarriageIO io;
  private CarriageIOInputsAutoLogged inputs = new CarriageIOInputsAutoLogged();

  /** Creates a new CarriageSubsystem. */
  public CarriageSubsystem(CarriageIO carriageIO) {
    io = carriageIO;
  }
  
  public void beamBreak(){
    if (inputs.detected == true) {
      io.setCarriageVolts(0);
    }
  }

  public Command setVolts(DoubleSupplier volts) {
    return new RunCommand(()->{
      double speed = volts.getAsDouble();
      Logger.recordOutput("DEBUG/Carriage/LeftTrigger", speed);
      this.setSpeed(speed);
    }, this);
  }

  public void setSpeed(double volts){
    io.setCarriageVolts(volts);
  }

  public void settoZero() {
    io.settoZero();
  }

  @Override
  public void periodic() {
    io.processInputs(inputs);
    Logger.processInputs("Carriage", inputs);
  }
}
