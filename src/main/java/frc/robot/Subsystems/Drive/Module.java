package frc.robot.Subsystems.drive;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Meters;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;

public class Module {
    private final int index;
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
    private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(DriveConstants.driveS, DriveConstants.driveV, DriveConstants.driveA);


    public Module(ModuleIO io, int index) {
        this.io = io;
        this.index = index;

    }

    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
    
        // Calculate positions for odometry
        int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
        odometryPositions = new SwerveModulePosition[sampleCount];
        for (int i = 0; i < sampleCount; i++) {
          double positionMeters =
              inputs.odometryDrivePositionsRad[i]
                  * DriveConstants.wheelRadius.in(Meter);
          Rotation2d angle = inputs.odometryTurnPositions[i];
          odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
    }

    // Returns the Angle of the Module
    public Rotation2d getAngle() {
        return inputs.turnPosition;
    }

    // Returns the drive position in Meters
    public double getPositionMeters() {
        return (inputs.drivePosition);
    }

    public void setState(SwerveModuleState state){
        io.setDriveMotor(state.speedMetersPerSecond, driveFeedforward.calculate(state.speedMetersPerSecond));
        io.setTurnMotor(state.angle.getRadians());
    }

    public double getVelocityMetersPerSec() {
        return inputs.driveVelocity;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getPositionMeters(), getAngle());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
    }

    public void runCharacterization(double volts){
        io.runCharacterization(volts);
    }

    public void setBrakeMode(boolean enabled) {
        io.setBrakeMode(enabled);
    }

    public double[] getOdometryTimestamps(){
        return inputs.odometryTimestamps;
    }

    public SwerveModulePosition[] getOdometryPositions(){
        return odometryPositions;
    }

}