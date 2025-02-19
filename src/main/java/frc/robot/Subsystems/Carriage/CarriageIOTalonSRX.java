package frc.robot.Subsystems.Carriage;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import frc.robot.Constants.CarriageConstants;

public class CarriageIOTalonSRX implements CarriageIO {
    private final TalonSRX carriage = new TalonSRX(CarriageConstants.carriageId);

    public CarriageIOTalonSRX() {
        carriage.configFactoryDefault();

        carriage.setInverted(true);
    }

    @Override
    public void processInputs(CarriageIOInputs inputs) {
        inputs.carriageRPM = carriage.getSelectedSensorVelocity();
        inputs.carriagesVolts = carriage.getMotorOutputVoltage();
        inputs.carriageAmps = carriage.getSupplyCurrent();
        inputs.carriageTemp = carriage.getTemperature();

    }

    @Override
    public void setCarriageVolts(double volts) {
        carriage.set(ControlMode.PercentOutput, volts);
    }

    @Override
    public void setCarriageRPM(double rPM) {
        carriage.set(ControlMode.PercentOutput, rPM);
    }

    @Override
    public void settoZero() {
        carriage.set(ControlMode.PercentOutput, 0);
    }

    @Override
    public void setCarriagePID(double kP, double kI, double kD) {
        
    }
}
