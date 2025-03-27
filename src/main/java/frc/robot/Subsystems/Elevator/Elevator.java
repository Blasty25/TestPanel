// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.ExponentialProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;

/** Add your docs here. */
public class Elevator extends SubsystemBase {

    private LoggedTunableNumber maxVelocity = new LoggedTunableNumber("Elevator/Constraints/MaxVelocity", 4.2);
    private LoggedTunableNumber maxAccerlation = new LoggedTunableNumber("Elevator/Constraints/maxAcceleration", 3.5);

    private ElevatorIO io;
    private ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/PID/kP", 40.0);
    private LoggedTunableNumber kI = new LoggedTunableNumber("Elevator/PID/kI", 0.0);
    private LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/PID/kD", 27.0);

    private LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/FF/kS", 3.0);
    private LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/FF/kG", 0.0);
    private LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/FF/kV", 4.2);
    private LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/FF/kA", 3.3);

    private ElevatorFeedforward ff = new ElevatorFeedforward(kS.getAsDouble(), kG.getAsDouble(), kV.getAsDouble(),
            kA.getAsDouble(), 0.02);

    private Distance difference = Meters.zero();
    private double tolerance = ElevatorConstants.tolerance;

    private ProfiledPIDController pid = new ProfiledPIDController(kP.getAsDouble(), kI.getAsDouble(), kD.getAsDouble(),
            new TrapezoidProfile.Constraints(maxVelocity.getAsDouble(), maxAccerlation.getAsDouble()));

    public Elevator(ElevatorIO io) {
        this.io = io;
        pid.setTolerance(0.1);
    }

    public void setSetpoint(String setpoint) {
        if (setpoint.equals("STOW")) {
            calculateVolts(Inches.of(0).in(Meters), 0);
        }
        if (setpoint.equals("L1")) {
            calculateVolts(Inches.of(10).in(Meters),0);
        }
        if (setpoint.equals("L2")) {
            calculateVolts(Inches.of(16).in(Meters),0);
        }
        if (setpoint.equals("L3")) {
            calculateVolts(Inches.of(24).in(Meters),0);
        }
        if (setpoint.equals("L4")) {
            calculateVolts(Inches.of(36).in(Meters),0);
        }
    }

    public void calculateVolts(double position, double velocity) {
        pid.setGoal(new State(position, velocity));
        inputs.targetHeight = Meters.of(pid.getGoal().position);
        inputs.setpoint = pid.getGoal().position;

        double pidOutput = pid.calculate(inputs.currentHeight.in(Meters), inputs.setpoint);
        double ffOutput = ff.calculateWithVelocities(inputs.velocity.in(MetersPerSecond), 0.27);

        Logger.recordOutput("Elevator/PIDOutput", pidOutput);
        Logger.recordOutput("Elevator/FFOutput", ffOutput * Math.signum(pidOutput));

        double output = pidOutput + ffOutput * Math.signum(pidOutput);
        io.setVolts(pidOutput + ffOutput);

        // if (difference.in(Meters) < tolerance) {
        //     ffOutput = 0;
        //     io.setVolts(output + (ffOutput * (Math.signum(pidOutput))));
        // }
    }

    public Command resetEncoder() {
        return Commands.runOnce(() -> {
            io.resetEncoder();
        }, this);
    }

    public Command runSetpoint(String position) {
        return Commands.run(() -> {
            String setpoint = position;
            this.setSetpoint(setpoint);
        }, this);
    }

    @Override
    public void periodic() {
        Logger.processInputs("Elevator", inputs);
        io.updateInputs(inputs);
        if (kP.hasChanged(hashCode())) {
            pid.setP(kP.getAsDouble());
        }

        if (kI.hasChanged(hashCode())) {
            pid.setI(kI.getAsDouble());
        }

        if (kD.hasChanged(hashCode())) {
            pid.setD(kD.getAsDouble());
        }
        difference = (inputs.targetHeight.minus(inputs.currentHeight));
        Logger.recordOutput("Elevator/TargetHeight", inputs.targetHeight);
    }
}
