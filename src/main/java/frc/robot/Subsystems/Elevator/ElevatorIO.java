package frc.robot.Subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorInputs{
        public double position = 0.0;
        public double velocity = 0.0;
        public double leftVoltage = 0.0;
        public double rightVoltage = 0.0;
        public double leftCurrent = 0.0;
        public double rightCurrent = 0.0;
        public double simVoltage = 0.0;
        public double simCurrent = 0.0;

        public double[] voltages = new double[] {};
    }

    public default void updateInputs(ElevatorInputs inputs) {}

    public default void setHeight(double position, double feedForward) {}

    public default void reset() {}

}
