package frc.robot.Subsystems.drive.pathfinding;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.drive.Drive;

import org.littletonrobotics.junction.Logger;
import static frc.robot.Constants.*;
public class leftPIDAllign extends Command {
  private final Drive drive;
  private final PIDController xPID;
  private final PIDController yPID;
  private final PIDController thetaPID;
  private PoseAllignment poseAllignment = new PoseAllignment();
  private Pose2d target;
  private boolean alliance;

  public leftPIDAllign(Drive drive, boolean alliance) {
    this.drive = drive;
    this.alliance = alliance;
    this.xPID = new PIDController(10.0, 0.0, 0.5);
    this.yPID = new PIDController(10.0, 0.0, 0.03);
    this.thetaPID = new PIDController(10.0, 0.0, 0.0);

    addRequirements(drive);
  }

  @Override
  public void initialize() {
    // Reset PID controllers to prevent accumulated error
    xPID.reset();
    yPID.reset();
    thetaPID.reset();
    if (alliance) {
      this.target = drive.getPose().nearest(poseAllignment.redLeft); // Set target position
    } else{
      this.target = drive.getPose().nearest(poseAllignment.blueLeft);
    }

    xPID.setTolerance(0.05);
    yPID.setTolerance(0.05);
    thetaPID.setTolerance(0.05);
  }

  @Override
  public void execute() {
    Pose2d robotPose = drive.getPose(); // Get updated robot pose

    // Cal PID outputs
    ChassisSpeeds zoom = ChassisSpeeds.fromFieldRelativeSpeeds(
    xPID.calculate(robotPose.getX(), target.getX()),
    yPID.calculate(robotPose.getY(), target.getY()),
    thetaPID.calculate(robotPose.getRotation().getRadians(), target.getRotation().getRadians()),
    drive.getRotation());

    // zoom zoom
    drive.runVelocity(ChassisSpeeds.discretize(zoom, 0.02));

    // Log data for debugging
    Logger.recordOutput("Drive/PID/Target", target);
    Logger.recordOutput("Drive/PID/RobotPose", robotPose);
    Logger.recordOutput("Drive/PID/XSetpoint", xPID.getSetpoint());
    Logger.recordOutput("Drive/PID/YSetpoint", yPID.getSetpoint());
    // Logger.recordOutput("Drive/PID/ThetaSetpoint", thetaPID.getSetpoint());
    System.out.println(ChassisSpeeds.discretize(zoom, 0.02));
  }

  @Override
  public boolean isFinished() {
    boolean setpoint = xPID.atSetpoint() && yPID.atSetpoint() && thetaPID.atSetpoint();
    boolean joystickOveride = Math.abs(DriveConstants.controller.getLeftX()) > 0.5
        || Math.abs(DriveConstants.controller.getLeftY()) > 0.5
        || Math.abs(DriveConstants.controller.getRightX()) > 0.5;
    return setpoint || joystickOveride;
  }

  @Override
  public void end(boolean interrupted) {
    drive.runVelocity(new ChassisSpeeds());
  }
}
