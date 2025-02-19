// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Elevator extends SubsystemBase {
    private final ElevatorIO io;
    private ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    private final TrapezoidProfile profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(2.5, 4.5));
    private TrapezoidProfile.State profileState = new TrapezoidProfile.State(0.0, 0.0);
    private TrapezoidProfile.State futureProfileState = new TrapezoidProfile.State(0.0, 0.0);

    private ElevatorFeedforward feedforward = new ElevatorFeedforward(ElevatorConstants.kS, ElevatorConstants.kG, ElevatorConstants.kV, ElevatorConstants.kA);

    private final SysIdRoutine routine;

    public Elevator(ElevatorIO io) {
        this.io = io;

        routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Velocity.ofRelativeUnits(1.0, Units.Volts.per(Units.Seconds)), 
                Voltage.ofRelativeUnits(3.0, Units.Volts), 
                Time.ofRelativeUnits(20.0, Units.Seconds)
            ), 
            new SysIdRoutine.Mechanism(
                voltage -> io.setHeight(0, voltage.magnitude()),
                log -> {
                    log.motor("elevator")
                        .voltage(Voltage.ofRelativeUnits(inputs.voltages[0], Units.Volts))
                        .linearPosition(Distance.ofRelativeUnits(inputs.position, Units.Meters))
                        .linearVelocity(LinearVelocity.ofRelativeUnits(inputs.velocity, Units.MetersPerSecond));
                },
                this
            )
        );
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);
    }

    @AutoLogOutput(key="Elevator/Position/Measured")
    public double getPosition() {
        return inputs.position;
    }

    public void setPosition(double position) {
        futureProfileState = profile.calculate(0.02, profileState, new TrapezoidProfile.State(position, 0.0));
        Logger.recordOutput("Elevator/Position/Setpoint", profileState.position);
        Logger.recordOutput("Elevator/Velocity/Setpoint", profileState.velocity);
        double feedforwardValue = feedforward.calculateWithVelocities(profileState.velocity, futureProfileState.velocity);
        Logger.recordOutput("Elevator/Feedforward", feedforwardValue);
        io.setHeight(profileState.position, feedforwardValue);
        profileState = futureProfileState;
    }

    @AutoLogOutput(key="Elevator/Velocity/Measured")
    public double getVelocity() {
        return inputs.velocity;
    }

    public void reset() {
        io.reset();
    }

    public Command sysIdRoutine() {
        return Commands.sequence(
            routine.quasistatic(SysIdRoutine.Direction.kForward).until(() -> inputs.position > 0.9),
            routine.quasistatic(SysIdRoutine.Direction.kReverse).until(() -> inputs.position < 0.1),
            routine.dynamic(SysIdRoutine.Direction.kForward).until(() -> inputs.position > 0.9),
            routine.dynamic(SysIdRoutine.Direction.kReverse).until(() -> inputs.position < 0.1)
        );
    }
}
