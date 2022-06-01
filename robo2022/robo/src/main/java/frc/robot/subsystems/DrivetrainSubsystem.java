package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.Constants;
//import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DrivetrainSubsystem extends SubsystemBase {
    private static final double MAX_VOLTAGE = 12.0;
    public static final double MAX_VELOCITY_METERS_PER_SECOND = 4.14528;
    public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;




    //private final PigeonIMU gyroscope = new PigeonIMU(Constants.DRIVETRAIN_PIGEON_ID);
    private final AHRS m_navx = new AHRS(SPI.Port.kMXP, (byte) 200); // NavX connected over MXP

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0),
            new Translation2d(-Constants.DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -Constants.DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    //private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(gyroscope.getFusedHeading()));
    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, Rotation2d.fromDegrees(m_navx.getYaw()));

    
    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

    public DrivetrainSubsystem() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");

        // frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
        //         shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
        //                 .withSize(2, 4)
        //                 .withPosition(0, 0),
        //         Mk4iSwerveModuleHelper.GearRatio.L3,
        //         Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
        //         Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
        //         Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
        //         Constants.FRONT_LEFT_MODULE_STEER_OFFSET
        // );

        // frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
        //         shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
        //                 .withSize(2, 4)
        //                 .withPosition(2, 0),
        //         Mk4iSwerveModuleHelper.GearRatio.L3,
        //         Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
        //         Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
        //         Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
        //         Constants.FRONT_RIGHT_MODULE_STEER_OFFSET
        // );

        // backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
        //         shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
        //                 .withSize(2, 4)
        //                 .withPosition(4, 0),
        //         Mk4iSwerveModuleHelper.GearRatio.L3,
        //         Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
        //         Constants.BACK_LEFT_MODULE_STEER_MOTOR,
        //         Constants.BACK_LEFT_MODULE_STEER_ENCODER,
        //         Constants.BACK_LEFT_MODULE_STEER_OFFSET
        // );

        // backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
        //         shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
        //                 .withSize(2, 4)
        //                 .withPosition(6, 0),
        //         Mk4iSwerveModuleHelper.GearRatio.L3,
        //         Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
        //         Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
        //         Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
        //         Constants.BACK_RIGHT_MODULE_STEER_OFFSET
        // );


        frontLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                shuffleboardTab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                Mk4iSwerveModuleHelper.GearRatio.L1,
                Constants.FRONT_LEFT_MODULE_DRIVE_MOTOR,
                Constants.FRONT_LEFT_MODULE_STEER_MOTOR,
                Constants.FRONT_LEFT_MODULE_STEER_ENCODER,
                Constants.FRONT_LEFT_MODULE_STEER_OFFSET
        );

        frontRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                shuffleboardTab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk4iSwerveModuleHelper.GearRatio.L1,
                Constants.FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.FRONT_RIGHT_MODULE_STEER_MOTOR,
                Constants.FRONT_RIGHT_MODULE_STEER_ENCODER,
                Constants.FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        backLeftModule = Mk4iSwerveModuleHelper.createFalcon500(
                shuffleboardTab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk4iSwerveModuleHelper.GearRatio.L1,
                Constants.BACK_LEFT_MODULE_DRIVE_MOTOR,
                Constants.BACK_LEFT_MODULE_STEER_MOTOR,
                Constants.BACK_LEFT_MODULE_STEER_ENCODER,
                Constants.BACK_LEFT_MODULE_STEER_OFFSET
        );

        backRightModule = Mk4iSwerveModuleHelper.createFalcon500(
                shuffleboardTab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk4iSwerveModuleHelper.GearRatio.L1,
                Constants.BACK_RIGHT_MODULE_DRIVE_MOTOR,
                Constants.BACK_RIGHT_MODULE_STEER_MOTOR,
                Constants.BACK_RIGHT_MODULE_STEER_ENCODER,
                Constants.BACK_RIGHT_MODULE_STEER_OFFSET
        );

        shuffleboardTab.addNumber("Gyroscope Angle", () -> getRotation().getDegrees());
        shuffleboardTab.addNumber("Pose X", () -> odometry.getPoseMeters().getX());
        shuffleboardTab.addNumber("Pose Y", () -> odometry.getPoseMeters().getY());

    }

    public void zeroGyroscope() {
        // odometry.resetPosition(
        //         new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)),
        //         Rotation2d.fromDegrees(gyroscope.getFusedHeading())
        // );


        odometry.resetPosition(
                new Pose2d(odometry.getPoseMeters().getTranslation(), Rotation2d.fromDegrees(0.0)),
                Rotation2d.fromDegrees(m_navx.getYaw())
        );
        

    }

    public Rotation2d getRotation() {
        return odometry.getPoseMeters().getRotation();
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    @Override
    public void periodic() {
        odometry.update(Rotation2d.fromDegrees(m_navx.getYaw()),
                new SwerveModuleState(frontLeftModule.getDriveVelocity(), new Rotation2d(frontLeftModule.getSteerAngle())),
                new SwerveModuleState(frontRightModule.getDriveVelocity(), new Rotation2d(frontRightModule.getSteerAngle())),
                new SwerveModuleState(backLeftModule.getDriveVelocity(), new Rotation2d(backLeftModule.getSteerAngle())),
                new SwerveModuleState(backRightModule.getDriveVelocity(), new Rotation2d(backRightModule.getSteerAngle()))
        );

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);

        frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
    }
}