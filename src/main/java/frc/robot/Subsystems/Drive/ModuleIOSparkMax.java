// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;

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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class ModuleIOSparkMax implements ModuleIO {
    private SparkMax driveSparky;
    private SparkMax turnSparky;

    private AnalogEncoder encoder;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder turnEncoder;

    private SparkClosedLoopController driveController;
    private SparkClosedLoopController turnController;

    private SparkMaxConfig driveConfig = new SparkMaxConfig();
    private SparkMaxConfig turnConfig = new SparkMaxConfig();

    private double encoderOffset;


    public ModuleIOSparkMax(Config config){
        driveSparky = new SparkMax(config.driveMotorId, MotorType.kBrushless);
        turnSparky = new SparkMax(config.turnMotorId, MotorType.kBrushless);

        encoder = new AnalogEncoder(config.encoderChannel);

        encoderOffset = config.encoderOffset;

        driveConfig
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(DriveConstants.driveCurrentLimitAmps);
        driveConfig.encoder
        .positionConversionFactor(DriveConstants.drivePositionConversionFactor)
        .velocityConversionFactor(DriveConstants.driveVelocityFactor);
        driveConfig.closedLoop
        .pid(DriveConstants.drivekP, DriveConstants.drivekI, DriveConstants.drivekD);

        turnConfig.
        inverted(config.isTurnInverted)
        .idleMode(IdleMode.kCoast)
        .smartCurrentLimit(DriveConstants.turnCurrentLimitAmps);
        turnConfig.encoder
        .positionConversionFactor(DriveConstants.turnPositionConversionFactor)
        .velocityConversionFactor(DriveConstants.turnVelocityFactor);
        turnConfig.closedLoop
        .pid(DriveConstants.turnkP, DriveConstants.turnkI, DriveConstants.turnkD);
        turnConfig.closedLoop
        .positionWrappingInputRange(-Math.PI, Math.PI)
        .positionWrappingEnabled(true);
        
        driveSparky.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turnSparky.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        driveEncoder = driveSparky.getEncoder();
        turnEncoder = turnSparky.getEncoder();

        driveController = driveSparky.getClosedLoopController();
        turnController = turnSparky.getClosedLoopController();
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.turnEncoder = encoder.get() - Units.radiansToRotations(encoderOffset);

        inputs.driveAppliedVolts = driveSparky.getAppliedOutput();
        inputs.turnAppliedVolts = turnSparky.getAppliedOutput();
        
        inputs.driveCurrent = driveSparky.getOutputCurrent();
        inputs.turnCurrent = turnSparky.getOutputCurrent();

        inputs.drivePosition = driveEncoder.getPosition();
        inputs.turnPosition = new Rotation2d(turnEncoder.getPosition());

        inputs.driveVelocity = driveEncoder.getVelocity();
        inputs.turnVelocity = turnEncoder.getVelocity();

        inputs.driveVoltage = new double[] {driveSparky.getAppliedOutput() * driveSparky.getBusVoltage()};

    }

    @Override
    public void setDriveMotor(double positionRad, double feedForward) {
        driveController.setReference(positionRad, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedForward);
    }

    @Override
    public void setTurnMotor(Rotation2d rotation) {
        turnController.setReference(rotation.getRadians(), ControlType.kPosition);
    }

    // @Override
    // public void runCharacterization(double volts) {
    //     driveSparky.setVoltage(volts);
    // }


    @Override
    public void setBrakeMode(boolean enabled) {
        driveConfig.idleMode(
            enabled ? IdleMode.kCoast : IdleMode.kBrake
        );
        driveSparky.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
