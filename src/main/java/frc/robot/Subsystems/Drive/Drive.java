package frc.robot.Subsystems.Drive;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.DriveConstants;

/** Add your docs here. */
public class Drive extends SubsystemBase {
    private final Module[] modules = new Module[4];
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final ModuleIOInputsAutoLogged moduleInputs = new ModuleIOInputsAutoLogged();

    private Rotation2d gyroEstimator = new Rotation2d();

    private SwerveModulePosition[] firstPositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(DriveConstants.moduletranslations);
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, gyroInputs.yawHeading,
            firstPositions);
    private SysIdRoutine routine;

    public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;

        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);

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
                                    .linearPosition(Distance.ofRelativeUnits(moduleInputs.drivePosition,
                                            Units.Meters))
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
                                    .linearPosition(Distance.ofRelativeUnits(moduleInputs.turnPosition.getRotations(),
                                            Units.Meters))
                                    .linearVelocity(LinearVelocity.ofRelativeUnits(moduleInputs.turnVelocity,
                                            Units.MetersPerSecond));
                        },
                        this, "Turn"));

    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPose(pose);
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getStates());
    }

    public void runCharacterization(double volts) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(volts);
        }
    }

    @Override
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);

        for (Module module : modules) {
            module.updateInputs();
        }

        SwerveModulePosition[] currentState = modulePositions();

        SwerveModulePosition[] finalModules = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            finalModules[i] = new SwerveModulePosition(
                    currentState[i].distanceMeters - firstPositions[i].distanceMeters,
                    currentState[i].angle);
        }

        Twist2d twist = kinematics.toTwist2d(finalModules);
        gyroEstimator = gyroEstimator.plus(new Rotation2d(twist.dtheta));

        gyroInputs.RobotPose = odometry.update(new Rotation2d(), finalModules);

    }

    public SwerveModulePosition[] modulePositions() {
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            modulePositions[i] = modules[i].getPosition();
        }
        return modulePositions;
    }

    @AutoLogOutput(key = "Drive/Modules/SetpointsOptimized")
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        Logger.recordOutput("Drive/Modules/SetpointsOptimized", states);
        return states;
    }

    public Command fieldOriantedDrive(DoubleSupplier xSupplier, DoubleSupplier ySupplier, DoubleSupplier zSupplier) {
        return new RunCommand(
                () -> {
                    SwerveModuleState[] swervemodulestate = kinematics.toSwerveModuleStates(
                            ChassisSpeeds.discretize(
                                    ChassisSpeeds.fromFieldRelativeSpeeds(
                                            (MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.15))
                                                    * DriveConstants.maxDriveSpeed,
                                            (MathUtil.applyDeadband((ySupplier.getAsDouble()), 0.15))
                                                    * DriveConstants.maxDriveSpeed,
                                            (MathUtil.applyDeadband(zSupplier.getAsDouble(), 0.15))
                                                    * DriveConstants.maxAngularspeed,
                                            gyroInputs.yawHeading),
                                    0.02));

                    for (int i = 0; i < 4; i++) {
                        modules[i].setState(swervemodulestate[i]);
                    }
                }, this);
    }

    public void autoDrive(ChassisSpeeds speeds) {
        SwerveModuleState[] state = kinematics.toSwerveModuleStates(speeds);
        for (int i = 0; i < 4; i++) {
            modules[i].setState(state[i]);
        }
    }

    public Command sysIDSwerve() {
        return Commands.sequence(
                routine.quasistatic(SysIdRoutine.Direction.kForward).until(() -> moduleInputs.drivePosition > 1),
                routine.quasistatic(SysIdRoutine.Direction.kReverse).until(() -> moduleInputs.drivePosition < 0.1),
                routine.dynamic(SysIdRoutine.Direction.kForward).until(() -> moduleInputs.drivePosition > 1),
                routine.dynamic(SysIdRoutine.Direction.kReverse).until(() -> moduleInputs.drivePosition < 0.1));
    }

}