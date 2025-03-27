package frc.robot.Subsystems.drive;

import java.util.Random;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.DriveConstants;

public class ModuleIOSim implements ModuleIO {

        private DCMotorSim driveMotor;
        private DCMotorSim turnMotor;

        private double driveAppliedVolts = 0.0;
        private double turnAppliedVolts = 0.0;

        private double driveFF = 0.0;



        private PIDController drivePID = new PIDController(DriveConstants.drivekP, 0.0,
                        DriveConstants.drivekD);

        private PIDController turnPID = new PIDController(DriveConstants.turnkP, 0.0,
                        DriveConstants.turnkD);
 
        public ModuleIOSim() {
                driveMotor = new DCMotorSim(
                                LinearSystemId.createDCMotorSystem(DriveConstants.motor, DriveConstants.driveMOI,
                                                DriveConstants.driveGearing),
                                DriveConstants.motor);

                turnMotor = new DCMotorSim(
                                LinearSystemId.createDCMotorSystem(DriveConstants.motor, DriveConstants.turnMOI,
                                                DriveConstants.turnGearing),
                                DriveConstants.motor);
                turnPID.enableContinuousInput(-Math.PI, Math.PI);
        }

        @Override
        public void updateInputs(ModuleIOInputs inputs) {

                driveMotor.update(0.02);
                turnMotor.update(0.02);

                turnAppliedVolts = turnPID.calculate(turnMotor.getAngularPositionRad());
                driveAppliedVolts = driveFF + drivePID.calculate(driveMotor.getAngularVelocityRadPerSec());

                // Updating Module Values
                inputs.driveCurrent = driveMotor.getCurrentDrawAmps();
                inputs.turnCurrent = turnMotor.getCurrentDrawAmps();

                inputs.drivePosition = driveMotor.getAngularPositionRad();
                inputs.driveVelocity = driveMotor.getAngularVelocityRadPerSec();

                inputs.turnPosition = new Rotation2d(turnMotor.getAngularPositionRad());
                inputs.turnVelocity = turnMotor.getAngularVelocityRadPerSec();

                // Set Simulation Stuff
                turnMotor.setInputVoltage(MathUtil.clamp(turnAppliedVolts, -12, 12));
                driveMotor.setInputVoltage(MathUtil.clamp(driveAppliedVolts, -12, 12));

                inputs.driveAppliedVolts = driveAppliedVolts;
                inputs.turnAppliedVolts = turnAppliedVolts;

                // Update odometry inputs (50Hz because high-frequency odometry in sim doesn't
                // matter)
                inputs.odometryTimestamps = new double[] { Timer.getFPGATimestamp() };
                inputs.odometryDrivePositionsRad = new double[] { inputs.drivePosition };
                inputs.odometryTurnPositions = new Rotation2d[] { inputs.turnPosition };
        }

        @Override
        public void setDriveMotor(double velocity, double feedforward) {
                driveFF = feedforward;
                drivePID.setSetpoint(velocity);
        }

        @Override
        public void setTurnMotor(double rotation) {
                turnPID.setSetpoint(rotation);
        }
}
