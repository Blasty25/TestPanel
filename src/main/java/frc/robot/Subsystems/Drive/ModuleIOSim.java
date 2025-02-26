package frc.robot.Subsystems.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.DriveConstants;

public class ModuleIOSim implements ModuleIO {

        private DCMotorSim driveMotor = new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(DriveConstants.motor, DriveConstants.driveMOI,
                                        DriveConstants.driveGearing),
                        DriveConstants.motor);

        private DCMotorSim turnMotor = new DCMotorSim(
                        LinearSystemId.createDCMotorSystem(DriveConstants.motor, DriveConstants.turnMOI,
                                        DriveConstants.turnGearing),
                        DriveConstants.motor);

        private PIDController drivePID = new PIDController(DriveConstants.drivekP, DriveConstants.drivekI,
                        DriveConstants.drivekD);

        private PIDController turnPID = new PIDController(DriveConstants.turnkP, DriveConstants.turnkI,
                        DriveConstants.turnkD);

        private double driveVelocity;
        private Rotation2d turnPosition;

        private double driveAppliedVolts = 0.0;
        private double turnAppliedVolts = 0.0;

        private double driveFF = 0.0;

        public ModuleIOSim() {
                turnPID.enableContinuousInput(-Math.PI, Math.PI);
        }

        @Override
        public void updateInputs(ModuleIOInputs inputs) {

                driveMotor.update(0.02);
                turnMotor.update(0.02);

                driveVelocity = driveMotor.getAngularVelocityRadPerSec();
                inputs.turnVelocity = turnMotor.getAngularVelocityRadPerSec();

                inputs.driveAppliedVolts = driveMotor.getInputVoltage();
                inputs.turnAppliedVolts = turnMotor.getInputVoltage();

                turnAppliedVolts = turnPID.calculate(turnMotor.getAngularVelocityRadPerSec());
                driveAppliedVolts += driveFF + drivePID.calculate(driveMotor.getAngularVelocityRadPerSec());

                // Set Simulation Stuff
                turnMotor.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12, 12));
                driveMotor.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12, 12));

                // Updating Module Values
                inputs.driveCurrent = driveMotor.getCurrentDrawAmps();
                inputs.turnCurrent = turnMotor.getCurrentDrawAmps();

                inputs.drivePosition = driveMotor.getAngularPositionRad();
                inputs.driveVelocity = driveVelocity;

                turnPosition = new Rotation2d(turnMotor.getAngularPositionRad());

                inputs.turnPosition = turnPosition.div(DriveConstants.wheelRadius);
                inputs.turnVelocity = turnMotor.getAngularVelocityRadPerSec();
        }

        @Override
        public void setDriveMotor(double velocity, double acceleration) {
                driveFF = acceleration;
                drivePID.setSetpoint(velocity / DriveConstants.wheelRadius);
        }

        @Override
        public void setTurnMotor(Rotation2d rotation) {
                turnPID.setSetpoint(rotation.getRadians());
        }
}
