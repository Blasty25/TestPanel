// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


/** Add your docs here. */
public class ElevatorIOSparkMax implements ElevatorIO{
    private SparkMax leftMotor;
    private SparkMax rightMotor;

    private SparkMaxConfig config = new SparkMaxConfig();
    private SparkClosedLoopController controller;

    private RelativeEncoder encoder;
    
    public ElevatorIOSparkMax(){
        leftMotor = new SparkMax(31, MotorType.kBrushless);
        rightMotor = new SparkMax(32, MotorType.kBrushless);

        config
        .inverted(false)
        .idleMode(IdleMode.kCoast);
        config.encoder
        .positionConversionFactor(ElevatorConstants.positionConversionFactor)
        .velocityConversionFactor(ElevatorConstants.velocityConversionFactor);
        config.closedLoop
        .pid(ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);

        leftMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.follow(leftMotor, true);
        rightMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = leftMotor.getEncoder();
        controller = leftMotor.getClosedLoopController();
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.position = encoder.getPosition();
        inputs.velocity = encoder.getVelocity();

        inputs.leftVoltage = leftMotor.getAppliedOutput();
        inputs.rightVoltage = rightMotor.getAppliedOutput();
        
        inputs.leftCurrent = leftMotor.getOutputCurrent();
        inputs.rightCurrent = rightMotor.getOutputCurrent();

        inputs.voltages = new double[] {leftMotor.getAppliedOutput() * leftMotor.getBusVoltage() , rightMotor.getAppliedOutput() * rightMotor.getBusVoltage()};
    }

    @Override
    public void setHeight(double position, double feedForward) {
        controller.setReference(position, ControlType.kPosition, ClosedLoopSlot.kSlot0, feedForward);
    }

    @Override
    public void reset() {
        encoder.setPosition(0);
    }
}
