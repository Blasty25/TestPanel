// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.drive;

import static edu.wpi.first.units.Units.Value;

import java.util.Queue;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveDrivetrain.OdometryThread;
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
import frc.robot.Subsystems.drive.util.SparkOdometryThread;
import frc.robot.util.LoggedTunableNumber;

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
    private ModuleIOInputsAutoLogged input = new ModuleIOInputsAutoLogged();

    public ModuleIOSparkMax(Config config) {

        driveSparky = new SparkMax(config.driveMotorId, MotorType.kBrushless);
        turnSparky = new SparkMax(config.turnMotorId, MotorType.kBrushless);

        encoder = new AnalogEncoder(config.encoderChannel);

        driveController = driveSparky.getClosedLoopController();
        turnController = turnSparky.getClosedLoopController();

        encoderOffset = config.encoderOffset;

        driveConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(DriveConstants.driveCurrentLimitAmps)
                .inverted(true);
        driveConfig.encoder
                .positionConversionFactor(DriveConstants.drivePositionConversionFactor)
                .velocityConversionFactor(DriveConstants.driveVelocityFactor);
        driveConfig.closedLoop
                .pid(DriveConstants.drivekP, DriveConstants.drivekI, DriveConstants.drivekD);

        turnConfig.inverted(config.isTurnInverted)
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(DriveConstants.turnCurrentLimitAmps);
        turnConfig.encoder
                .positionConversionFactor(DriveConstants.turnPositionConversionFactor)
                .velocityConversionFactor(DriveConstants.turnVelocityFactor);
        turnConfig.closedLoop
                .pid(DriveConstants.turnkP, DriveConstants.turnkI, DriveConstants.turnkD)
                .positionWrappingInputRange(-Math.PI, Math.PI)
                .positionWrappingEnabled(true);

        driveEncoder = driveSparky.getEncoder();
        turnEncoder = turnSparky.getEncoder();

        turnEncoder.setPosition(encoder.get() - encoderOffset);
        driveSparky.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turnSparky.configure(turnConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        inputs.noOffsetAbs = encoder.get();
        inputs.absPosition = encoder.get() - encoderOffset;

        inputs.drivePosition = driveEncoder.getPosition();
        inputs.driveVelocity = driveEncoder.getVelocity();
        inputs.driveAppliedVolts = driveSparky.getAppliedOutput() * driveSparky.getBusVoltage();
        inputs.driveCurrent = driveSparky.getOutputCurrent();
        inputs.driveVoltage = new double[] { driveSparky.getAppliedOutput() * driveSparky.getBusVoltage() };

        inputs.turnPosition = Rotation2d.fromRotations(turnEncoder.getPosition());
        inputs.turnAppliedVolts = turnSparky.getAppliedOutput() * turnSparky.getBusVoltage();
        inputs.turnCurrent = turnSparky.getOutputCurrent();
        inputs.turnVelocity = turnEncoder.getVelocity();
    }

    @Override
    public void setDriveMotor(double positionMPS, double feedForward) {
        driveController.setReference(positionMPS, ControlType.kVelocity, ClosedLoopSlot.kSlot0, feedForward);
    }

    @Override
    public void setTurnMotor(double rotation, double ffVoltage) {
        Logger.recordOutput("Drive/Debug/TurnSetpoint", rotation);
        turnController.setReference(rotation, ControlType.kPosition, ClosedLoopSlot.kSlot0, ffVoltage);
    }

    @Override
    public void runCharacterization(double volts) {
        if (input.runSysId) {
            driveSparky.setVoltage(volts);
            // turnSparky.setVoltage(volts);
        }
    }

    @Override
    public void setBrakeMode(boolean enabled) {
        driveConfig.idleMode(
                enabled ? IdleMode.kCoast : IdleMode.kBrake);
        driveSparky.configure(driveConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
}
