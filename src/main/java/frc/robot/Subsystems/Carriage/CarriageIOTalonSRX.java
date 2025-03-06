package frc.robot.Subsystems.Carriage;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class CarriageIOTalonSRX implements CarriageIO {
    private final TalonSRX carriage = new TalonSRX(CarriageConstants.carriageId);

    public CarriageIOTalonSRX() {
        carriage.configFactoryDefault();

        carriage.setInverted(true);
    }

    @Override
    public void processInputs(CarriageIOInputs inputs) {
        inputs.carriagesVolts = carriage.getMotorOutputVoltage();
        inputs.carriageAmps = carriage.getSupplyCurrent();
        inputs.carriageTemp = carriage.getTemperature();

    }

    @Override
    public void setCarriageVolts(double volts) {
        carriage.set(ControlMode.PercentOutput, volts);
    }

    @Override
    public void settoZero() {
        carriage.set(ControlMode.PercentOutput, 0);
    }
}
