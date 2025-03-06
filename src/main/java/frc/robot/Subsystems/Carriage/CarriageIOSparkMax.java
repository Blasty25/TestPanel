// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Carriage;

import com.ctre.phoenix.motorcontrol.IFollower;
import com.reduxrobotics.sensors.canandcolor.Canandcolor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class CarriageIOSparkMax implements CarriageIO {

    private final SparkMax carriage = new SparkMax(CarriageConstants.carriageId, SparkMax.MotorType.kBrushless);
    private Canandcolor canandcolor = new Canandcolor(CarriageConstants.colorID);

    public CarriageIOSparkMax() {
        SparkMaxConfig config = new SparkMaxConfig();
        canandcolor.resetFactoryDefaults();
        config.inverted(true);
        config.idleMode(IdleMode.kBrake);
        carriage.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        carriage.set(0.1);
    }

    @Override
    public void processInputs(CarriageIOInputs inputs) {
        inputs.carriageAmps = carriage.getOutputCurrent();
        inputs.carriagesVolts = carriage.getAppliedOutput() * carriage.getBusVoltage();
        inputs.carriageTemp = carriage.getMotorTemperature();

        inputs.sensorConnected = canandcolor.isConnected();
        inputs.sensorTemp = canandcolor.getTemperature();
        inputs.sensorRange = canandcolor.getProximity();
        
        if (canandcolor.getProximity() < 0.1) {
            inputs.detected = true;
        } else {
            inputs.detected = false;
        }
    }

    @Override
    public void setCarriageVolts(double volts) {
        carriage.set(MathUtil.clamp(volts, -1.0, 1.0));
    }

    @Override
    public void settoZero() {
        carriage.set(0);
    }

}
