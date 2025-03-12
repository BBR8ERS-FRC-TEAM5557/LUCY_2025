package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.swerve.SwerveModuleConstantsFactory;
import edu.wpi.first.math.util.Units;

import static frc.robot.Constants.RobotMap.*;
import static frc.robot.Constants.Physical.*;

import frc.robot.Constants;

public class SwerveConstants {
        public static final String kSubsystemName = "Swerve";

        public static final double kDriveGearRatio = (50.0 / 16.0) * (19.0 / 25.0) * (45.0 / 15.0);
        public static final double kSteerGearRatio = 150.0 / 7.0;
        public static final double kWheelRadiusMeters = Units.inchesToMeters(2.00);
        private static final boolean kInvertSteer = true;
        private static final boolean kInvertDrive = true;
        private static final boolean kInvertEncoder = false;

        // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
        // This may need to be tuned to your individual robot
        private static final double kCoupleRatio = (50.0 / 16.0);

        public static final double kWheelCOF = 1.5;

        // These are only used for simulation
        private static final double kSteerInertia = 0.00001;
        private static final double kDriveInertia = 0.001;

        /* Swerve Profiling Values */
        public static final double kTheoreticalMaxSpeed = kWheelRadiusMeters * 2 * Math.PI
                        * ((Constants.Physical.kKrakenFreeSpeed / 60.0) / kDriveGearRatio); // meters per second
        public static final double kTrueMaxSpeed = kTheoreticalMaxSpeed * 0.9; // Max out at 90% to estimate true max
                                                                               // speed

        public static final double kTheoreticalMaxOmega = kTheoreticalMaxSpeed / kModuleRadius; // radians per second
        public static final double kTrueMaxOmega = kTrueMaxSpeed / kModuleRadius;

        /* MOTION PLANNER PID */
        public class PID {
                public static final double kTranslationkP = 5.0;
                public static final double kTranslationkI = 0.0;
                public static final double kTranslationkD = 0.0;

                public static final double kRotationkP = 5.0;
                public static final double kRotationkI = 0.0;
                public static final double kRotationkD = 0.0;

                public static final double kSnapMaxOmega = kTrueMaxOmega * 0.65;
                public static final double kSnapMaxAlpha = kSnapMaxOmega / 0.25;
        }

        public record KinematicLimits(double maxLinearSpeed, double maxLinearAccel, double maxOmega, double maxAlpha) {
        };

        // KINEMATIC LIMITS
        public class Kinematics {
                public static final KinematicLimits kUncappedLimits = new KinematicLimits(
                                kTheoreticalMaxSpeed,
                                Double.MAX_VALUE,
                                kTheoreticalMaxOmega,
                                Double.MAX_VALUE);

                public static final KinematicLimits kTeleopLimits = new KinematicLimits(
                                kTrueMaxSpeed,
                                kTrueMaxSpeed / 0.2,
                                kTrueMaxOmega * 0.7,
                                Double.MAX_VALUE);

                public static final KinematicLimits kAutoLimits = new KinematicLimits(
                                kTheoreticalMaxSpeed,
                                kTheoreticalMaxSpeed / 0.2,
                                kTrueMaxOmega * 0.5,
                                Double.MAX_VALUE);
        }

        public class TunerConstants {
                private static final String kCANbusName = "canivore";

                // The closed-loop output type to use for the steer motors;
                // This affects the PID/FF gains for the steer motors
                private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;

                // The closed-loop output type to use for the drive motors;
                // This affects the PID/FF gains for the drive motors
                private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

                private static final SteerFeedbackType feedbackSource = SteerFeedbackType.FusedCANcoder;

                // The stator current at which the wheels start to slip;
                // This needs to be tuned to your individual robot
                private static final double kSlipCurrentA = 80.0;

                // Theoretical free speed (m/s) at 12v applied output;
                // This needs to be tuned to your individual robot
                private static final double kSpeedAt12VoltsMps = kTrueMaxSpeed;

                // Simulated voltage necessary to overcome friction
                private static final double kSteerFrictionVoltage = 0.0;
                private static final double kDriveFrictionVoltage = 0.0;

                // The steer motor uses any SwerveModule.SteerRequestType control request with
                // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
                private static final Slot0Configs steerGains = new Slot0Configs()
                                .withKP(50)
                                .withKI(0)
                                .withKD(0) // .2
                                .withKS(0)
                                .withKV(0)
                                .withKA(0);

                // When using closed-loop control, the drive motor uses the control
                // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
                private static final Slot0Configs driveGains = new Slot0Configs()
                                .withKP(0.35)
                                .withKI(0)
                                .withKD(0)
                                .withKS(0)
                                .withKV((12.0 - kDriveFrictionVoltage) / (kSpeedAt12VoltsMps
                                                * (kDriveGearRatio / (2 * Math.PI * kWheelRadiusMeters))))
                                .withKA(0);

                private static final Pigeon2Configuration Pigeon2 = new Pigeon2Configuration();

                public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
                                .withPigeon2Id(kPigeon)
                                .withPigeon2Configs(Pigeon2)
                                .withCANBusName(kCANbusName);

                private static final TalonFXConfiguration initialDriveConfigs = new TalonFXConfiguration();
                static {
                        initialDriveConfigs.CurrentLimits.SupplyCurrentLimit = 80;
                        initialDriveConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
                        initialDriveConfigs.Audio.BeepOnBoot = false;
                        initialDriveConfigs.Audio.BeepOnConfig = false;
                        initialDriveConfigs.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.01;
                        initialDriveConfigs.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.01;
                        initialDriveConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.01;
                }
                private static final TalonFXConfiguration initialSteerConfigs = new TalonFXConfiguration();
                static {
                        initialSteerConfigs.Audio.BeepOnBoot = false;
                        initialSteerConfigs.Audio.BeepOnConfig = false;
                        initialSteerConfigs.CurrentLimits.StatorCurrentLimit = 50.0;
                        initialSteerConfigs.CurrentLimits.StatorCurrentLimitEnable = false;
                }

                private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator = new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
                                .withDriveMotorGearRatio(kDriveGearRatio)
                                .withSteerMotorGearRatio(kSteerGearRatio)
                                .withWheelRadius(kWheelRadiusMeters)
                                .withSlipCurrent(kSlipCurrentA)
                                .withSteerMotorGains(steerGains)
                                .withDriveMotorGains(driveGains)
                                .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
                                .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
                                .withSpeedAt12Volts(kSpeedAt12VoltsMps)
                                .withSteerFrictionVoltage(kSteerFrictionVoltage)
                                .withDriveFrictionVoltage(kDriveFrictionVoltage)
                                .withSteerInertia(kSteerInertia)
                                .withDriveInertia(kDriveInertia)
                                .withFeedbackSource(feedbackSource)
                                .withCouplingGearRatio(kCoupleRatio)
                                .withDriveMotorInitialConfigs(initialDriveConfigs)
                                .withSteerMotorInitialConfigs(initialSteerConfigs);

                public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft = ConstantCreator
                                .createModuleConstants(
                                                kFLTurnMotor,
                                                kFLDriveMotor,
                                                kFLEncoder,
                                                kFLOffset.getRotations(),
                                                kTrackWidth / 2,
                                                kWheelBase / 2,
                                                kInvertDrive,
                                                kInvertSteer,
                                                kInvertEncoder);

                public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight = ConstantCreator
                                .createModuleConstants(
                                                kFRTurnMotor,
                                                kFRDriveMotor,
                                                kFREncoder,
                                                kFROffset.getRotations(),
                                                kTrackWidth / 2,
                                                -kWheelBase / 2,
                                                kInvertDrive,
                                                kInvertSteer,
                                                kInvertEncoder);

                public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft = ConstantCreator
                                .createModuleConstants(
                                                kBLTurnMotor,
                                                kBLDriveMotor,
                                                kBLEncoder,
                                                kBLOffset.getRotations(),
                                                -kTrackWidth / 2,
                                                kWheelBase / 2,
                                                kInvertDrive,
                                                kInvertSteer,
                                                kInvertEncoder);

                public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight = ConstantCreator
                                .createModuleConstants(
                                                kBRTurnMotor,
                                                kBRDriveMotor,
                                                kBREncoder,
                                                kBROffset.getRotations(),
                                                -kTrackWidth / 2,
                                                -kWheelBase / 2,
                                                kInvertDrive,
                                                kInvertSteer,
                                                kInvertEncoder);
        }
}
