package frc.robot.Subsystems.drive.pathfinding;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.drive.Drive;
import frc.robot.Constants.DriveConstants;
import org.littletonrobotics.junction.Logger;

public class PIDAlign extends Command {
  private final Drive drive;
  private final PIDController xPID;
  private final PIDController yPID;
  private final ProfiledPIDController thetaPID;
  private PoseAllignment poseAllignment = new PoseAllignment();
  private Pose2d target;

  public PIDAlign(Drive drive) {
    this.drive = drive;
    this.xPID = new PIDController(0.6, 0.0, 0.5);
    this.yPID = new PIDController(0.5, 0.0, 0.03);
    this.thetaPID = new ProfiledPIDController(1.0, 0.0, 0.0, new Constraints(4.0, 3.5));

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Reset PID controllers to prevent accumulated error
    xPID.reset();
    yPID.reset();
    thetaPID.reset(0);
    this.target = drive.getPose().nearest(poseAllignment.redLeft); // Set target position

    xPID.setTolerance(0.05);
    yPID.setTolerance(0.05);
    thetaPID.setTolerance(0.05);
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose(); // Get updated robot pose

    // Cal PID outputs
    double xSpeed = xPID.calculate(robotPose.getX(), target.getX());
    double ySpeed = yPID.calculate(robotPose.getY(), target.getY());
    double thetaSpeed = thetaPID.calculate(robotPose.getRotation().getRadians(), target.getRotation().getRadians());

    //zoom zoom
    drive.autoDrive(new ChassisSpeeds(xSpeed, ySpeed, thetaSpeed));

    // Log data for debugging
    Logger.recordOutput("Drive/PID/Target", target);
    Logger.recordOutput("Drive/PID/RobotPose", robotPose);
    Logger.recordOutput("Drive/PID/XSetpoint", xPID.getSetpoint());
    Logger.recordOutput("Drive/PID/YSetpoint", yPID.getSetpoint());
    // Logger.recordOutput("Drive/PID/ThetaSetpoint", thetaPID.getSetpoint());
  }

  @Override
  public boolean isFinished() {
    boolean setpoint =  xPID.atSetpoint() && yPID.atSetpoint() && thetaPID.atSetpoint();
    boolean joystickOveride = Math.abs(DriveConstants.controller.getLeftX()) > 0.5 || Math.abs(DriveConstants.controller.getLeftY()) > 0.5 || Math.abs(DriveConstants.controller.getRightX()) > 0.5;
    return setpoint || joystickOveride;
  }

  @Override
  public void end(boolean interrupted) {
    drive.autoDrive(new ChassisSpeeds()); 
  }
}
