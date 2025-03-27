// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Subsystems.elevator.ElevatorConstants.*;

public class ElevatorIOSim implements ElevatorIO{

    private ElevatorSim elevator;

    private double volts = 0.0;

    public ElevatorIOSim() {
        this.elevator = new ElevatorSim(
                LinearSystemId.createElevatorSystem(gearbox, elevatorMass, drumRadius, gearing),
                gearbox, minHeight, maxHeight, simGravity, minHeight);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        elevator.update(0.02);
        inputs.leftVolts = Volts.of(volts);
        inputs.currentHeight = Meters.of(elevator.getPositionMeters());
        inputs.velocity = MetersPerSecond.of(elevator.getVelocityMetersPerSecond());

        inputs.leftCurrent = Amps.of(elevator.getCurrentDrawAmps());
    }

    @Override
    public void setVolts(double voltage) {
        this.volts = voltage;
        elevator.setInputVoltage(MathUtil.clamp(voltage, -12, 12));
    }

    @Override
    public void resetEncoder() {
        elevator.setInputVoltage(0);
    }

    
}
