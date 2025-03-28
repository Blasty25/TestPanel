// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.Spark;


/** Add your docs here. */
public class ElevatorIOReal implements ElevatorIO{
    private SparkMaxConfig config = new SparkMaxConfig();

    private SparkMax leftSparky = new SparkMax(ElevatorConstants.leftID, MotorType.kBrushless);
    private SparkMax rightSparky = new SparkMax(ElevatorConstants.rightID, MotorType.kBrushless);

    public RelativeEncoder encoder;

    public ElevatorIOReal(){
        config
        .idleMode(IdleMode.kCoast)
        .inverted(false)
        .voltageCompensation(12);

        config.encoder
        .positionConversionFactor(ElevatorConstants.positionConversionFactor)
        .velocityConversionFactor(ElevatorConstants.positionConversionFactor / 60);

        config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

        leftSparky.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.follow(leftSparky, true);
        
        rightSparky.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
        encoder = leftSparky.getEncoder();
        encoder.setPosition(0.0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.currentHeight = Meters.ofBaseUnits(encoder.getPosition());

        inputs.leftCurrent = Amps.of(leftSparky.getOutputCurrent());
        inputs.rightCurrent = Amps.of(rightSparky.getOutputCurrent());

        inputs.leftVolts = Volts.of(leftSparky.getAppliedOutput() * leftSparky.getBusVoltage());
        inputs.rightVolts = Volts.of(rightSparky.getAppliedOutput() * rightSparky.getBusVoltage());

        inputs.velocity = MetersPerSecond.of(encoder.getVelocity());
        Logger.recordOutput("Elevator/RawRotations", leftSparky.getEncoder().getPosition());
    }

    @Override
    public void setVolts(double volts) {
        leftSparky.setVoltage(volts);
    }

    @Override
    public void resetEncoder() {
        encoder.setPosition(0.0);
    }
}
