// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Carriage;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.reduxrobotics.sensors.canandcolor.*;

public class CarriageSubsystem extends SubsystemBase {
  
  CarriageIO io;
  CarriageIOInputsAutoLogged inputs = new CarriageIOInputsAutoLogged();
  private Canandcolor color;

  private static CarriageSubsystem instance;

  /** Creates a new CarriageSubsystem. */
  public CarriageSubsystem(CarriageIO carriageIO) {
    io = carriageIO;
    instance = this;
    color = new Canandcolor(9);
  }

public static CarriageSubsystem getInstance() {
    if (instance == null) {
      instance = new CarriageSubsystem(new CarriageIOSim());
    }
    return instance;
  }

  public ColorData getColor(){
    return color.getColor();
  }
  
  public void beamBreak(){
    double finalColor = color.getBlue() + color.getGreen() + color.getRed();
    while (finalColor >= 720 && finalColor <= 775) {
      io.setCarriageVolts(0.1);
    }
    io.setCarriageVolts(0);
  }

  
  public void setVolts(double volts) {
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
