package frc.robot.Subsystems.Carriage;

import org.littletonrobotics.junction.AutoLog;


public interface CarriageIO {

    @AutoLog
    public class CarriageIOInputs {
        public double carriageRPM = 0.0;

        public double targetRPM = 0.0;

        public double carriageAmps = 0.0;

        public double carriagesVolts = 0.0;

        public double carriagekP = 0.0;
        public double carriagekI = 0.0;
        public double carriagekD = 0.0;

        public double carriageTemp = 0.0;

    }

    public abstract void processInputs(CarriageIOInputs inputs);

    public abstract void setCarriageVolts(double volts);

    public abstract void setCarriageRPM(double rPM);

    public abstract void settoZero();

    public abstract void setCarriagePID(double kP, double kI, double kD);
}
