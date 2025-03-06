package frc.robot.Subsystems.Carriage;

import org.littletonrobotics.junction.AutoLog;

public interface CarriageIO {

    @AutoLog
    public class CarriageIOInputs {
        public double carriageAmps = 0.0;
        public double carriagesVolts = 0.0;
        public double carriageTemp = 0.0;

        public boolean sensorConnected = false;
        public double sensorRange = 0.0;
        public double sensorTemp = 0.0;
        public boolean detected = false;

    }

    public abstract void processInputs(CarriageIOInputs inputs);

    public abstract void setCarriageVolts(double volts);

    public abstract void settoZero();
}
