package frc.robot.Subsystems.Elevator;

import static frc.robot.Subsystems.Elevator.ElevatorConstants.*;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.ExponentialProfile;
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
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private final ExponentialProfile profile = new ExponentialProfile(ExponentialProfile.Constraints.fromCharacteristics(maxProfileVoltage - s - g, v, a));
    private ExponentialProfile.State profileState = new ExponentialProfile.State(0.0, 0.0);
    private ExponentialProfile.State futureProfileState = new ExponentialProfile.State(0.0, 0.0);

    private ElevatorFeedforward feedforward = new ElevatorFeedforward(s, g, v);

    private final SysIdRoutine routine;

    public Elevator(ElevatorIO io) {
        this.io = io;

        periodic();

        profileState.position = inputs.position;

        routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Velocity.ofRelativeUnits(sysIdRampUp, Units.Volts.per(Units.Seconds)), 
                Voltage.ofRelativeUnits(sysIdStep, Units.Volts), 
                Time.ofRelativeUnits(sysIdTimeout, Units.Seconds)
            ), 
            new SysIdRoutine.Mechanism(
                voltage -> io.setPosition(0, voltage.magnitude()),
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
        futureProfileState = profile.calculate(0.02, profileState, new ExponentialProfile.State(position, 0.0));
        Logger.recordOutput("Elevator/Position/Setpoint", profileState.position);
        Logger.recordOutput("Elevator/Velocity/Setpoint", profileState.velocity);
        double feedforwardValue = feedforward.calculateWithVelocities(profileState.velocity, futureProfileState.velocity);
        Logger.recordOutput("Elevator/Feedforward", feedforwardValue);
        io.setPosition(futureProfileState.position, feedforwardValue);
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
            routine.quasistatic(SysIdRoutine.Direction.kForward).until(() -> inputs.position > sysIdMaxPosition),
            routine.quasistatic(SysIdRoutine.Direction.kReverse).until(() -> inputs.position < sysIdMinPosition),
            routine.dynamic(SysIdRoutine.Direction.kForward).until(() -> inputs.position > sysIdMaxPosition),
            routine.dynamic(SysIdRoutine.Direction.kReverse).until(() -> inputs.position < sysIdMinPosition)
        );
    }
}