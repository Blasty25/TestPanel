// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Drive;



import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class ModuleIOTalonFX implements ModuleIO {

    // HardWare
    private final TalonFX driveTalon;
    private final TalonFX turnTalon;
    private final CANcoder encoder;

    // Config
    private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration turnConfig = new TalonFXConfiguration();
    private final double encoderOffset;
    private final boolean isTurnInverted;

    // Control requests
    private final PositionTorqueCurrentFOC positionTorqueCurrentRequest = new PositionTorqueCurrentFOC(0.0)
            .withUpdateFreqHz(0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest = new VelocityTorqueCurrentFOC(0.0)
            .withUpdateFreqHz(0);

    // Inputs from drive motor
    private final StatusSignal<Angle> drivePosition;
    private final StatusSignal<AngularVelocity> driveVelocity;
    private final StatusSignal<Voltage> driveAppliedVolts;
    private final StatusSignal<Current> driveCurrent;

    // Inputs from turn motor
    private final double turnAbsolutePosition;
    private final StatusSignal<Angle> turnPosition;
    private final StatusSignal<AngularVelocity> turnVelocity;
    private final StatusSignal<Voltage> turnAppliedVolts;
    private final StatusSignal<Current> turnCurrent;

    public ModuleIOTalonFX(Config config) {
        driveTalon = new TalonFX(config.driveMotorId);
        turnTalon = new TalonFX(config.turnMotorId);
        encoder = new CANcoder(config.encoderChannel);
        isTurnInverted = config.isTurnInverted;
        encoderOffset = config.encoderOffset;

        // Config Drive Motor
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.Slot0 = new Slot0Configs().withKP(DriveConstants.drivekP).withKI(DriveConstants.drivekI)
                .withKD(DriveConstants.drivekD);
        driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.driveGearing;
        // Max and min Draw amps
        driveConfig.TorqueCurrent.PeakForwardTorqueCurrent = 80;
        driveConfig.TorqueCurrent.PeakReverseTorqueCurrent = -60;
        driveConfig.CurrentLimits.StatorCurrentLimit = 80;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        //////////////////////////////////////////////////////////////////
        driveConfig.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.02;
        driveTalon.getConfigurator().apply(driveConfig, 0.25);
        driveTalon.setPosition(0.0, 0.25);

        // Config Turn Motor
        turnConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        turnConfig.Slot0 = new Slot0Configs().withKP(DriveConstants.turnkP).withKI(DriveConstants.turnkI)
                .withKD(DriveConstants.turnkD);
        turnConfig.Feedback.FeedbackRemoteSensorID = encoder.getDeviceID();
        turnConfig.Feedback.RotorToSensorRatio = DriveConstants.turnGearing;
        turnConfig.ClosedLoopGeneral.ContinuousWrap = true;
        turnConfig.TorqueCurrent.PeakForwardTorqueCurrent = 40;
        turnConfig.TorqueCurrent.PeakReverseTorqueCurrent = -40;
        turnConfig.CurrentLimits.StatorCurrentLimit = 40;
        turnConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        turnConfig.MotorOutput.Inverted = isTurnInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        turnTalon.getConfigurator().apply(turnConfig, 0.25);

        // Create drive status signals
        drivePosition = driveTalon.getPosition();
        driveVelocity = driveTalon.getVelocity();
        driveAppliedVolts = driveTalon.getMotorVoltage();
        driveCurrent = driveTalon.getStatorCurrent();

        // Create turn status signals
        turnAbsolutePosition = encoder.getPosition().getValueAsDouble() - encoderOffset;
        turnPosition = turnTalon.getPosition();
        turnVelocity = turnTalon.getVelocity();
        turnAppliedVolts = turnTalon.getMotorVoltage();
        turnCurrent = turnTalon.getStatorCurrent();

        BaseStatusSignal.setUpdateFrequencyForAll(250, drivePosition, turnPosition);
        // DEUBG: make sure turning is getting updated
        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                driveVelocity,
                driveAppliedVolts,
                driveCurrent,
                turnVelocity,
                turnAppliedVolts,
                turnCurrent);

        ParentDevice.optimizeBusUtilizationForAll(driveTalon, turnTalon);
    }

    @Override
    public void updateInputs(ModuleIOInputs inputs) {
        // Update Drive Inputs
        inputs.drivePosition = Units.rotationsToRadians(drivePosition.getValueAsDouble());
        inputs.driveVelocity = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
        inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
        inputs.driveCurrent = driveCurrent.getValueAsDouble();

        // Update Turn Inputs
        inputs.turnEncoder = turnAbsolutePosition;
        inputs.turnPosition = Rotation2d.fromRotations(turnPosition.getValueAsDouble());
        inputs.turnVelocity = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
        inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
        inputs.turnCurrent = turnCurrent.getValueAsDouble();
    }

    @Override
    public void setDriveMotor(double positionRad, double feedForward) {
        driveTalon.setControl(
                velocityTorqueCurrentRequest
                        .withVelocity(Units.radiansToRotations(positionRad))
                        .withFeedForward(feedForward));
    }

    @Override
    public void runCharacterization(double volts) {
        //run motors at different times! DO NOT RUN DRIVE AND TURN AT SAME TIME! Do 2 different SysId tests
        driveTalon.setVoltage(volts);
        // turnTalon.setVoltage(volts);
    }

    @Override
    public void runTurnPosition(Rotation2d rotation) {
        turnTalon.setControl(positionTorqueCurrentRequest.withPosition(rotation.getRotations()));
    }
}
