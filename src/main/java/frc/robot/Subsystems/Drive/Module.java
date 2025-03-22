package frc.robot.Subsystems.Drive;

import static edu.wpi.first.units.Units.Inches;
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
                  * DriveConstants.wheelRadius.in(Inches);
          Rotation2d angle = inputs.odometryTurnPositions[i];
          odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
        }
    }

    // Returns the Angle of the Module
    public Rotation2d getAngle() {
        return new Rotation2d(inputs.turnPosition);
    }

    // Returns the drive position in Meters
    public double getPositionMeters() {
        return (inputs.drivePosition);
    }

    public void setState(SwerveModuleState state){
      state.optimize(getAngle());
      state.cosineScale(getAngle());
      io.setDriveMotor(state.speedMetersPerSecond / DriveConstants.wheelRadius.in(Inches), driveFeedforward.calculate(state.speedMetersPerSecond));
      io.setTurnMotor(state.angle.getRadians());
      Logger.recordOutput("Drive/Optimzied", state);
    }

    public void xState(SwerveModuleState state){
        state.optimize(getAngle());
        state.cosineScale(new Rotation2d(inputs.turnPosition));
        io.setTurnMotor(state.angle.getRadians());
        io.setDriveMotor(state.speedMetersPerSecond / DriveConstants.wheelRadius.in(Meters), driveFeedforward.calculate(state.speedMetersPerSecond));
      }

    public double getVelocityMetersPerSec() {
        return inputs.driveVelocity * DriveConstants.wheelRadius.in(Inches);
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