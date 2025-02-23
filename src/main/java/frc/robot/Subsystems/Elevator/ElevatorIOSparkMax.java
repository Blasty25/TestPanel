package frc.robot.Subsystems.Elevator;

import static frc.robot.Subsystems.Elevator.ElevatorConstants.*;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

public class ElevatorIOSparkMax implements ElevatorIO {
    private final SparkMax leftMotor;
    private final SparkMax rightMotor;

    private final RelativeEncoder encoder;
    private final SparkClosedLoopController feedback;

    public ElevatorIOSparkMax() {
        leftMotor = new SparkMax(31, MotorType.kBrushless);
        rightMotor = new SparkMax(32, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config
            .inverted(false)
            .smartCurrentLimit(60)
            .idleMode(IdleMode.kCoast);
        config.encoder
            .positionConversionFactor(positionConversionFactor)
            .velocityConversionFactor(velocityConversionFactor);
        config.closedLoop
            .p(p)
            .i(i)
            .d(d);
        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config
            .follow(leftMotor, true);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = leftMotor.getEncoder();
        feedback = leftMotor.getClosedLoopController();
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.position = encoder.getPosition();
        inputs.velocity = encoder.getVelocity();
        inputs.voltages = new double[] {leftMotor.getAppliedOutput() * leftMotor.getBusVoltage() , rightMotor.getAppliedOutput() * rightMotor.getBusVoltage()};
        inputs.currents = new double[] {leftMotor.getOutputCurrent(), rightMotor.getOutputCurrent()};
    }

    @Override
    public void setPosition(double position, double ffVoltage) {
        feedback.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVoltage);
    }

    @Override
    public void reset() {
        encoder.setPosition(0.0);
    }
}