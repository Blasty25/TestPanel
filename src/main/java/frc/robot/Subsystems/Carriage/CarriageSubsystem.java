// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Carriage;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CarriageSubsystem extends SubsystemBase {
  
  CarriageIO io;
  CarriageIOInputsAutoLogged inputs = new CarriageIOInputsAutoLogged();

  private static CarriageSubsystem instance;

  /** Creates a new CarriageSubsystem. */
  public CarriageSubsystem(CarriageIO carriageIO) {
    io = carriageIO;
    instance = this;
  }

public static CarriageSubsystem getInstance() {
    if (instance == null) {
      instance = new CarriageSubsystem(new CarriageIOSim());
    }
    return instance;
  }

  
  public void setRPM(double rPM) {
    io.setCarriageRPM(rPM);
  }

  public void settoZero() {
    io.settoZero();;
  }

  @Override
  public void periodic() {
    
    io.processInputs(inputs);
    Logger.processInputs("Carriage", inputs);
  }
}
