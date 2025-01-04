package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import static frc.robot.Constants.RobotMap.*;
import static frc.robot.Constants.Physical.*;

import frc.robot.Constants;

public class SwerveConstants {
    public static final String kSubsystemName = "Swerve";

    private static final double kDriveGearRatio = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);
    private static final double kSteerGearRatio = 150.0 / 7.0;
    private static final double kWheelRadiusInches = 1.9135;
    private static final boolean kInvertSteer = true;
    private static final boolean kInvertDrive = true;
    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatio = 3.5714285714285716;

    // These are only used for simulation
    private static final double kSteerInertia = 0.00001;
    private static final double kDriveInertia = 0.001;

    /* Swerve Profiling Values */
    public static final double kTheoreticalMaxSpeed = kWheelRadiusInches * 2 * Math.PI
            * ((Constants.Physical.kKrakenFreeSpeed / 60.0) / kDriveGearRatio); // meters per second
    public static final double kTheoreticalMaxAcceleration = 7.090; // m/s^2
    public static final double kTheoreticalMaxOmega = 11.5; // radians per second

    public static final double kTrueMaxSpeed = kTheoreticalMaxSpeed * 0.8; // Max out at 85% to make sure speeds are
                                                                           // attainable (4.6 mps)
    public static final double kTrueMaxAcceleration = kTheoreticalMaxAcceleration * 0.8;
    public static final double kTrueMaxOmega = kTheoreticalMaxOmega * 0.85;

    /* MOTION PLANNER PID */
    public class PID {
        public static final double kTranslationkP = 5.0;
        public static final double kTranslationkI = 0.0;
        public static final double kTranslationkD = 0.0;

        public static final double kRotationkP = 4.0;
        public static final double kRotationkI = 0.0;
        public static final double kRotationkD = 0.0;

        public static final double kSnapMaxOmega = kTrueMaxOmega * 0.65;
        public static final double kSnapMaxAlpha = kSnapMaxOmega / 0.15;
    }

    public record KinematicLimits(double maxLinearSpeed, double maxLinearAccel, double maxOmega, double maxAlpha) {
    };

    // KINEMATIC LIMITS
    public class Kinematics {
        public static final KinematicLimits kUncappedLimits = new KinematicLimits(kTheoreticalMaxSpeed,
                Double.MAX_VALUE,
                kTheoreticalMaxOmega, Double.MAX_VALUE);

        public static final KinematicLimits kTeleopLimits = new KinematicLimits(kTheoreticalMaxSpeed,
                kTheoreticalMaxSpeed * 5.0,
                kTrueMaxOmega * 0.65, Double.MAX_VALUE);

        public static final KinematicLimits kAutoLimits = new KinematicLimits(kTheoreticalMaxSpeed,
                kTheoreticalMaxSpeed * 5.0,
                kTrueMaxOmega * 0.65, Double.MAX_VALUE);
    }

    public class TunerConstants {
        private static final String kCANbusName = "canivore";
        // The closed-loop output type to use for the steer motors;
        // This affects the PID/FF gains for the steer motors
        private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
        // The closed-loop output type to use for the drive motors;
        // This affects the PID/FF gains for the drive motors
        private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;
        // The stator current at which the wheels start to slip;
        // This needs to be tuned to your individual robot
        private static final double kSlipCurrentA = 70.0;
        // Theoretical free speed (m/s) at 12v applied output;
        // This needs to be tuned to your individual robot
        private static final double kSpeedAt12VoltsMps = kTheoreticalMaxSpeed;
        // The steer motor uses any SwerveModule.SteerRequestType control request with
        // the
        // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
        private static final Slot0Configs steerGains = new Slot0Configs()
                .withKP(100)
                .withKI(0)
                .withKD(0.05)
                .withKS(0.18)
                .withKV(1.5)
                .withKA(0);

        // When using closed-loop control, the drive motor uses the control
        // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
        private static final Slot0Configs driveGains = new Slot0Configs()
                .withKP(2.5)
                .withKI(0)
                .withKD(0)
                .withKS(0.78)
                .withKV(0)
                .withKA(0);

        private static final Pigeon2Configuration Pigeon2 = new Pigeon2Configuration();

        public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                .withPigeon2Id(kPigeon)
                .withPigeon2Configs(Pigeon2)
                .withCANbusName(kCANbusName);

        private static final SwerveModuleConstantsFactory ConstantCreator = new SwerveModuleConstantsFactory()
                .withDriveMotorGearRatio(kDriveGearRatio)
                .withWheelRadius(kWheelRadiusInches)
                .withSlipCurrent(kSlipCurrentA)
                .withSteerMotorGains(steerGains)
                .withDriveMotorGains(driveGains)
                .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
                .withSteerInertia(kSteerInertia)
                .withDriveInertia(kDriveInertia)
                .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
                .withCouplingGearRatio(kCoupleRatio);

        public static final SwerveModuleConstants FrontLeft = ConstantCreator
                .withSteerMotorInverted(kInvertSteer)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .createModuleConstants(
                        kFLTurnMotor,
                        kFLDriveMotor,
                        kFLEncoder,
                        kFLOffset.getRotations(),
                        kTrackWidth / 2,
                        kWheelBase / 2,
                        kInvertDrive);

        public static final SwerveModuleConstants FrontRight = ConstantCreator
                .withSteerMotorInverted(kInvertSteer)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .createModuleConstants(
                        kFRTurnMotor,
                        kFRDriveMotor,
                        kFREncoder,
                        kFROffset.getRotations(),
                        -kTrackWidth / 2,
                        kWheelBase / 2,
                        kInvertDrive);

        public static final SwerveModuleConstants BackLeft = ConstantCreator
                .withSteerMotorInverted(kInvertSteer)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .createModuleConstants(
                        kBLTurnMotor,
                        kBLDriveMotor,
                        kBLEncoder,
                        kBLOffset.getRotations(),
                        kTrackWidth / 2,
                        -kWheelBase / 2,
                        kInvertDrive);

        public static final SwerveModuleConstants BackRight = ConstantCreator
                .withSteerMotorInverted(kInvertSteer)
                .withSteerMotorGearRatio(kSteerGearRatio)
                .createModuleConstants(
                        kBRTurnMotor,
                        kBRDriveMotor,
                        kBREncoder,
                        kBROffset.getRotations(),
                        -kTrackWidth / 2,
                        -kWheelBase / 2,
                        kInvertDrive);
    }
}
