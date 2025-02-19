package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Constants {

  public static boolean kIsReal = Robot.isReal();
  public static boolean kTuningMode = true;

  public class RobotMap {
    public static final int kPigeon = 5;

    public static final int kFLDriveMotor = 22;
    public static final int kFLTurnMotor = 21;
    public static final int kFLEncoder = 11;
    public static final Rotation2d kFLOffset = Rotation2d.fromDegrees(157.1484375);

    public static final int kFRDriveMotor = 24;
    public static final int kFRTurnMotor = 23;
    public static final int kFREncoder = 12;
    public static final Rotation2d kFROffset = Rotation2d.fromDegrees(135.0);

    public static final int kBLDriveMotor = 28;
    public static final int kBLTurnMotor = 27;
    public static final int kBLEncoder = 14;
    public static final Rotation2d kBLOffset = Rotation2d.fromDegrees(-105.0);

    public static final int kBRDriveMotor = 26;
    public static final int kBRTurnMotor = 25;
    public static final int kBREncoder = 13;
    public static final Rotation2d kBROffset = Rotation2d.fromDegrees(-187.0);
  }

  public class Physical {
    // Physical Constants
    public static final double kChassisLength = Units.inchesToMeters(26.0);
    public static final double kChassisWidth = Units.inchesToMeters(26.0);
    public static final double kModuleInset = Units.inchesToMeters(2.625);

    public static final double kWheelBase = kChassisLength - (2 * kModuleInset);
    public static final double kTrackWidth = kChassisWidth - (2 * kModuleInset);
    public static final double kModuleRadius = Math.hypot(kWheelBase / 2.0, kTrackWidth / 2.0);

    public static final double kKrakenFreeSpeed = 6000;
    public static final double kKrakenFreeSpeedFOC = 5800;

    public static final double kMass = Units.lbsToKilograms(101.0);
    public static final double kMOI = (kMass / 12.0) * Math.hypot(kChassisLength, kChassisWidth); // estimate using
                                                                                                  // formula for a slab
  }

  public class Joystick {
    public static double kSteerJoystickDeadband = 0.05;
  }


  public static boolean disableHAL = false;

  public static void disableHAL() {
    disableHAL = true;
  }

}
