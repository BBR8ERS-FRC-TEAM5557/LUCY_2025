package frc.robot;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Constants {

        public static boolean kIsReal = Robot.isReal();
        public static boolean kTuningMode = true;

        public class RobotMap {
                public static final int kPigeon = 5;

                public static final int kFLDriveMotor = 26;
                public static final int kFLTurnMotor = 25;
                public static final int kFLEncoder = 11;
                public static final Rotation2d kFLOffset = Rotation2d.fromDegrees(-343.916015625);

                public static final int kFRDriveMotor = 28;
                public static final int kFRTurnMotor = 27;
                public static final int kFREncoder = 12; 
                public static final Rotation2d kFROffset = Rotation2d.fromDegrees(-83.68359375);

                public static final int kBLDriveMotor = 24;
                public static final int kBLTurnMotor = 23;
                public static final int kBLEncoder = 14;
                public static final Rotation2d kBLOffset = Rotation2d.fromDegrees(-160.400390625);

                public static final int kBRDriveMotor = 22;
                public static final int kBRTurnMotor = 21;
                public static final int kBREncoder = 13;
                public static final Rotation2d kBROffset = Rotation2d.fromDegrees(-54.5921875);
        }
        public class Physical {
                // Physical Constants
                public static final double kChassisLength = Units.inchesToMeters(22.0);
                public static final double kChassisWidth = Units.inchesToMeters(22.0);
                public static final double kBumperThickness = Units.inchesToMeters(0.75 + 2.5);
                public static final double kRobotLength = kChassisLength + 2 * kBumperThickness;
                public static final double kRobotWidth = kChassisWidth + 2 * kBumperThickness;

                public static final double kModuleInset = Units.inchesToMeters(2.625);
                public static final double kWheelBase = kChassisLength - (2 * kModuleInset);
                public static final double kTrackWidth = kChassisWidth - (2 * kModuleInset);
                public static final double kModuleRadius = Math.hypot(kWheelBase / 2.0, kTrackWidth / 2.0);

                public static final double kKrakenFreeSpeed = 6000;
                public static final double kKrakenFreeSpeedFOC = 5800;

                public static final double kMass = Units.lbsToKilograms(107.0);
                public static final double kMOI = (kMass / 12.0) * Math.hypot(kChassisLength, kChassisWidth); // estimate
                                                                                                              // using
                                                                                                              // formula
                                                                                                              // for a
                                                                                                              // slab
        }

        public static final class VisionConstants {
                public static final String kLimelightName = "limelight";
                public static final double kLimelightMountAngleDegrees = 15.0; // Angle from horizontal
                public static final double kLimelightLensHeightMeters = Units.inchesToMeters(24.0);
                public static final double kLimelightMountOffsetX = 0.0; // Forward offset from robot center
                public static final double kLimelightMountOffsetY = 0.0; // Left/right offset from robot center
        
                // AprilTag field layout
                public static final AprilTagFields kAprilTagField = AprilTagFields.kDefaultField;
        
                // Vision measurement standard deviations
                public static final double kVisionTranslationStdDev = 0.5; // meters
                public static final double kVisionRotationStdDev = 0.5; // radians
        
                // Target alignment constants
                public static final double kMaxAlignAngularVelocity = Math.PI; // rad/s
                public static final double kAlignPositionTolerance = Units.degreesToRadians(2.0); // radians
                public static final double kAlignVelocityTolerance = Units.degreesToRadians(5.0); // rad/s
        
                // PID constants for rotation alignment
                public static final double kAlignP = 2.0;
                public static final double kAlignI = 0.0;
                public static final double kAlignD = 0.1;
            }

        public class Joystick {
                public static double kSteerJoystickDeadband = 0.05;
        }

        public static boolean disableHAL = false;

        public static void disableHAL() {
                disableHAL = true;
        }

}
