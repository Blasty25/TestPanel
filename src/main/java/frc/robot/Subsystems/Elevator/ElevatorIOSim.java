// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import static frc.robot.Subsystems.Elevator.ElevatorConstants.*;

/** Add your docs here. */
public class ElevatorIOSim implements ElevatorIO{

    private ElevatorSim sim = new ElevatorSim(
        LinearSystemId.createElevatorSystem(elevatorMotor, elevatorMass, elevatorRadius, gearing),
        elevatorMotor,
        0,
        Units.inchesToMeters(44),
        true,
        0);

    private PIDController pid = new PIDController(kP, kI, kD);
    private double feedForward = 0.0;

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        sim.update(0.02);

        inputs.position = sim.getPositionMeters();
        inputs.velocity = sim.getVelocityMetersPerSecond();

        inputs.simCurrent = sim.getCurrentDrawAmps();

        double output = feedForward + pid.calculate(sim.getPositionMeters());
        sim.setInputVoltage(MathUtil.clamp(output, -12, 12));
    }

    @Override
    public void setHeight(double position, double feedForward) {
        this.feedForward = feedForward;
        pid.setSetpoint(position);
    }
}
