package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static boolean kIsReal = Robot.isReal();
  public static boolean kTuningMode = false;

  public class RobotMap {
    public static final int kPigeon = 5;

    public static final int kFLDriveMotor = 12;
    public static final int kFLTurnMotor = 21;
    public static final int kFLEncoder = 22;
    public static final Rotation2d kFLOffset = Rotation2d.fromDegrees(0.0);

    public static final int kFRDriveMotor = 14;
    public static final int kFRTurnMotor = 22;
    public static final int kFREncoder = 22;
    public static final Rotation2d kFROffset = Rotation2d.fromDegrees(0.0);

    public static final int kBLDriveMotor = 13;
    public static final int kBLTurnMotor = 23;
    public static final int kBLEncoder = 22;
    public static final Rotation2d kBLOffset = Rotation2d.fromDegrees(0.0);

    public static final int kBRDriveMotor = 15;
    public static final int kBRTurnMotor = 24;
    public static final int kBREncoder = 22;
    public static final Rotation2d kBROffset = Rotation2d.fromDegrees(0.0);
  }

  public class Physical {
    // Physical Constants
    public static final double kChassisLength = Units.inchesToMeters(22.0);
    public static final double kChassisWidth = Units.inchesToMeters(22.0);
    public static final double kModuleInset = Units.inchesToMeters(2.625);

    public static final double kWheelBase = kChassisLength - 2 * kModuleInset;
    public static final double kTrackWidth = kChassisWidth - 2 * kModuleInset;

    public static final double kKrakenFreeSpeed = 6000;
    public static final double kKrakenFreeSpeedFOC = 5800;
  }

  public class Joystick {
    public static double kSteerJoystickDeadband = 0.05;
  }

}
