package frc.robot.Subsystems.Drive;


import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, gyroInputs.yawHeading, firstPositions);
    private SysIdRoutine routine;

    public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
        this.gyroIO = gyroIO;

        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 3);
        modules[3] = new Module(brModuleIO, 3);

        onEnable();

        // Set the Brake mode to each Module
        for (Module module : modules) {
            module.setBrakeMode(true);
        }

        routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Velocity.ofRelativeUnits(1.0, Units.Volts.per(Units.Seconds)),
                Voltage.ofRelativeUnits(3.0, Units.Volts),
                Time.ofRelativeUnits(20.0, Units.Seconds)
            ),


            new SysIdRoutine.Mechanism(
                voltage -> runCharacterization(2),
                log -> {
                    log.motor("fLDrive")
                    .linearPosition(Distance.ofRelativeUnits(moduleInputs.drivePosition, Units.Meters))
                    .linearVelocity(LinearVelocity.ofRelativeUnits(moduleInputs.driveVelocity, Units.MetersPerSecond));
                },
                this, "fl Drive")
        );




        routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Velocity.ofRelativeUnits(1.0, Units.Volts.per(Units.Seconds)),
                Voltage.ofRelativeUnits(3.0, Units.Volts),
                Time.ofRelativeUnits(20.0, Units.Seconds)
            ),


            new SysIdRoutine.Mechanism(
                voltage -> runCharacterization(2),
                log -> {
                    log.motor("fr Drive")
                    .linearPosition(Distance.ofRelativeUnits(moduleInputs.drivePosition, Units.Meters))
                    .linearVelocity(LinearVelocity.ofRelativeUnits(moduleInputs.driveVelocity, Units.MetersPerSecond));
                },
                this, "fr Drive")
        );


        routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Velocity.ofRelativeUnits(1.0, Units.Volts.per(Units.Seconds)),
                Voltage.ofRelativeUnits(3.0, Units.Volts),
                Time.ofRelativeUnits(20.0, Units.Seconds)
            ),


            new SysIdRoutine.Mechanism(
                voltage -> runCharacterization(2),
                log -> {
                    log.motor("bl Drive")
                    .linearPosition(Distance.ofRelativeUnits(moduleInputs.drivePosition, Units.Meters))
                    .linearVelocity(LinearVelocity.ofRelativeUnits(moduleInputs.driveVelocity, Units.MetersPerSecond));
                },
                this, "bl Drive")
        );


        routine = new SysIdRoutine(
            new SysIdRoutine.Config(
                Velocity.ofRelativeUnits(1.0, Units.Volts.per(Units.Seconds)),
                Voltage.ofRelativeUnits(3.0, Units.Volts),
                Time.ofRelativeUnits(20.0, Units.Seconds)
            ),


            new SysIdRoutine.Mechanism(
                voltage -> runCharacterization(2),
                log -> {
                    log.motor("br Drive")
                    .linearPosition(Distance.ofRelativeUnits(moduleInputs.drivePosition, Units.Meters))
                    .linearVelocity(LinearVelocity.ofRelativeUnits(moduleInputs.driveVelocity, Units.MetersPerSecond));
                },
                this, "br Drive")
        );
    }


    public void runCharacterization(double volts){
        for(int i = 0; i < 4; i++){
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

        SwerveModulePosition[] modulePositions = modulePositions();

        SwerveModulePosition[] finalModules = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++){
            finalModules[i] = new SwerveModulePosition(
            modulePositions[i].distanceMeters - firstPositions[i].distanceMeters,
            modulePositions[i].angle
            );
        }

        Twist2d twist = kinematics.toTwist2d(finalModules);
        gyroEstimator = gyroEstimator.plus(new Rotation2d(twist.dtheta));

        gyroInputs.RobotPose = odometry.update(gyroEstimator, modulePositions);
        
        if (DriverStation.isTeleopEnabled()) {
            onEnable();
        }

    }

    public SwerveModulePosition[] modulePositions(){
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        for(int i = 0; i < 4; i++){
            modulePositions[i] = modules[i].getPosition();
        }
        return modulePositions;
    }

    public SwerveModuleState[] enabledStates = new SwerveModuleState[] {
        new SwerveModuleState(0, new Rotation2d(3.0 * Math.PI / 4.0)),
        new SwerveModuleState(0, new Rotation2d(Math.PI / 4.0)),
        new SwerveModuleState(0, new Rotation2d(Math.PI / 4.0)),
        new SwerveModuleState(0, new Rotation2d(3.0 * Math.PI / 4.0))
    };


    public void onEnable(){
        for(int i = 0; i < 4; i++){
            modules[i].xState(enabledStates[i]);
        }
    }
    
    @AutoLogOutput(key = "Drive/Modules/SetpointsOptimized")
    public SwerveModuleState[] getStates(){
        SwerveModuleState[] states = new SwerveModuleState[4]; 
        for(int i = 0; i < 4; i++){
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
                                    (MathUtil.applyDeadband(xSupplier.getAsDouble(), 0.1))* DriveConstants.maxDriveSpeed ,(MathUtil.applyDeadband((ySupplier.getAsDouble()), 0.1)) * DriveConstants.maxDriveSpeed,
                                    (MathUtil.applyDeadband(zSupplier.getAsDouble(),0.1)) * DriveConstants.maxAngularspeed ,gyroInputs.yawHeading) ,0.02));

                        for(int i = 0; i < 4; i++){
                            modules[i].setState(swervemodulestate[i]);
                        }
                }, this);
    }

    

    public Command sysIDSwerve() {
        return Commands.sequence(
            routine.quasistatic(SysIdRoutine.Direction.kForward).until(() -> moduleInputs.drivePosition > 1),
            routine.quasistatic(SysIdRoutine.Direction.kReverse).until(() -> moduleInputs.drivePosition < 0.1),
            routine.dynamic(SysIdRoutine.Direction.kForward).until(()-> moduleInputs.drivePosition > 1),
            routine.dynamic(SysIdRoutine.Direction.kReverse).until(()-> moduleInputs.drivePosition < 0.1)
        );
    }


}