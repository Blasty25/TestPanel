package frc.robot.Subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;

import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;
import frc.robot.Subsystems.drive.pathfinding.PoseAllignment;
import frc.robot.Subsystems.drive.util.SparkOdometryThread;

/** Add your docs here. */
public class Drive extends SubsystemBase {
    public static final double DRIVE_BASE_RADIUS = Math.max(
            Math.max(
                    Math.hypot(
                            DriveConstants.posTrack,
                            DriveConstants.posTrack),
                    Math.hypot(
                            DriveConstants.posTrack,
                            DriveConstants.negTrack)),
            Math.max(
                    Math.hypot(
                            DriveConstants.negTrack,
                            DriveConstants.posTrack),
                    Math.hypot(
                            DriveConstants.negTrack,
                            DriveConstants.negTrack)));
    private final Module[] modules = new Module[4];
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged moduleInputs = new ModuleIOInputsAutoLogged();
    public static final Lock odometryLock = new ReentrantLock();
    private Rotation2d rawGyroRotation = new Rotation2d();
    private PoseAllignment poseAllignment = new PoseAllignment();
    private Rotation2d gyroEstimator = new Rotation2d();
    private SwerveModulePosition[] firstPositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.moduletranslations);
    private final SwerveDrivePoseEstimator pose = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation,
            firstPositions, new Pose2d());

    public TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            LinearVelocity.ofBaseUnits(4.30, MetersPerSecond),
            LinearAcceleration.ofBaseUnits(4.99, MetersPerSecondPerSecond))
            .setKinematics(kinematics)
            .setReversed(false)
            .setStartVelocity(0.3)
            .setEndVelocity(3.0);

    public HolonomicDriveController trajDriveController = new HolonomicDriveController(new PIDController(1, 0, 0),
            new PIDController(1, 0, 0), new ProfiledPIDController(1, 0, 0, new Constraints(1, 1)));

    // MODULE MAP USE FOR DEBUGGING
    /*
     * |
     * FL(0) | FR(1)
     * |
     * ------|------
     * |
     * BL(2) | BR(3)
     * |
     */

    private SysIdRoutine routine;

    public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;

        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);

        SparkOdometryThread.getInstance().start();

        this.autoDrive(new ChassisSpeeds());

        // AUTOS PATH PLANNER
        try {
            DriveConstants.config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getSpeeds,
                (speed, feedforward) -> autoDrive(speed),
                new PPHolonomicDriveController(
                        new PIDConstants(DriveConstants.drivekP, DriveConstants.drivekI, DriveConstants.drivekD),
                        new PIDConstants(DriveConstants.turnkP, DriveConstants.turnkI, DriveConstants.turnkD)),
                DriveConstants.config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);

        // Set the Brake mode to each Module
        for (Module module : modules) {
            module.setBrakeMode(true);
        }

        routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Velocity.ofRelativeUnits(1.0, Units.Volts.per(Units.Seconds)),
                        Voltage.ofRelativeUnits(3.0, Units.Volts),
                        Time.ofRelativeUnits(20.0, Units.Seconds)),

                new SysIdRoutine.Mechanism(
                        voltage -> runCharacterization(2),
                        log -> {
                            log.motor("Drive")
                                    .linearPosition(Distance.ofRelativeUnits(moduleInputs.drivePosition, Units.Meters))
                                    .linearVelocity(LinearVelocity.ofRelativeUnits(moduleInputs.driveVelocity,
                                            Units.MetersPerSecond));
                        },
                        this, "Drive"));

        routine = new SysIdRoutine(
                new SysIdRoutine.Config(
                        Velocity.ofRelativeUnits(1.0, Units.Volts.per(Units.Seconds)),
                        Voltage.ofRelativeUnits(3.0, Units.Volts),
                        Time.ofRelativeUnits(20.0, Units.Seconds)),

                new SysIdRoutine.Mechanism(
                        voltage -> runCharacterization(2),
                        log -> {
                            log.motor("Turn")
                                    .linearPosition(Distance.ofRelativeUnits(moduleInputs.drivePosition, Units.Meters))
                                    .linearVelocity(LinearVelocity.ofRelativeUnits(moduleInputs.driveVelocity,
                                            Units.MetersPerSecond));
                        },
                        this, "Turn"));

    }

    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getPose() {
        return pose.getEstimatedPosition();
    }

    public void resetPose(Pose2d newPose) {
        pose.resetPose(newPose);
    }

    public ChassisSpeeds getSpeeds() {
        ChassisSpeeds currentSpeed = kinematics.toChassisSpeeds(getStates());
        return currentSpeed;
    }

    public void runCharacterization(double volts) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(volts);
        }
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    @Override
    public void periodic() {
        odometryLock.lock();
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (Module module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        if (DriverStation.isTeleopEnabled()) {
            this.autoDrive(new ChassisSpeeds());
        }

        // Update odometry
        double[] sampleTimestamps = modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] = new SwerveModulePosition(
                        modulePositions[moduleIndex].distanceMeters
                                - firstPositions[moduleIndex].distanceMeters,
                        modulePositions[moduleIndex].angle);
                firstPositions[moduleIndex] = modulePositions[moduleIndex];
            }

            // Update gyro angle
            if (gyroInputs.isConnected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                Twist2d twist = kinematics.toTwist2d(moduleDeltas);
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            pose.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

    }

    public Rotation2d getHeading() {
        return gyroInputs.yawHeading;
    }

    public void getTranlastion() {
        Pose2d translatedPose = getPose();
        Logger.recordOutput("Drive/Recorded/Pose", translatedPose.getTranslation());
    }

    public Command recordPose() {
        return Commands.runOnce(() -> {
            getTranlastion();
        });
    }

    public SwerveModulePosition[] modulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modulePositions[i] = modules[i].getPosition();
        }
        return modulePositions;
    }

    @AutoLogOutput(key = "Drive/Output")
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public void setModule(SwerveModuleState[] optimizedState) {
        for (int i = 0; i < 4; i++) {
            modules[i].setState(optimizedState[i]);
        }
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return DriveConstants.maxDriveSpeed;
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    public Command joystickDrive(
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            DoubleSupplier deadbandSupplier,
            DoubleSupplier percentSupplier) {
        return Commands.run(
                () -> {
                    ChassisSpeeds zoom = ChassisSpeeds.fromFieldRelativeSpeeds(
                            MathUtil.applyDeadband(xSupplier.getAsDouble(), deadbandSupplier.getAsDouble())
                                    * percentSupplier.getAsDouble(),
                            MathUtil.applyDeadband(ySupplier.getAsDouble(), deadbandSupplier.getAsDouble())
                                    * percentSupplier.getAsDouble(),
                            MathUtil.applyDeadband(omegaSupplier.getAsDouble(), deadbandSupplier.getAsDouble())
                                    * percentSupplier.getAsDouble(),
                            rawGyroRotation);

                    this.autoDrive(zoom);
                },
                this);
    }

    public void autoDrive(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        Logger.recordOutput("Drive/Setpoint", states);

        for (int i = 0; i < 4; i++) {
            if (i == 2)
                continue;
            if (i == 1)
                continue;

            states[i].optimize(modules[i].getAngle());
            states[i].cosineScale(modules[i].getAngle());

            modules[i].setState(states[i]);
        }

        Logger.recordOutput("Drive/Optimized", states);
    }

    public Command resetGyro() {
        return Commands.runOnce(() -> {
            gyroIO.reset();
        }, this);
    }

    public Command sysIDSwerve() {
        return Commands.sequence(
                routine.quasistatic(SysIdRoutine.Direction.kForward).until(() -> moduleInputs.drivePosition > 1),
                routine.quasistatic(SysIdRoutine.Direction.kReverse).until(() -> moduleInputs.drivePosition < 0.1),
                routine.dynamic(SysIdRoutine.Direction.kForward).until(() -> moduleInputs.drivePosition > 1),
                routine.dynamic(SysIdRoutine.Direction.kReverse).until(() -> moduleInputs.drivePosition < 0.1));
    }

    public Command followTraj() {
        return new RunCommand(
                () -> {
                    Pose2d robotPose = pose.getEstimatedPosition();
                    Pose2d target = robotPose.nearest(poseAllignment.redLeft);

                    Trajectory traj = TrajectoryGenerator.generateTrajectory(
                            robotPose, List.of(), target, trajectoryConfig);

                    Trajectory.State desiredState = traj.sample(traj.getTotalTimeSeconds());
                    Logger.recordOutput("Drive/PID/Align", target);
                    ChassisSpeeds zoom = trajDriveController.calculate(robotPose, desiredState, target.getRotation());
                    this.autoDrive(zoom);
                    if (trajDriveController.atReference()) {
                        this.autoDrive(new ChassisSpeeds());
                    }
                },
                this);
    }

}