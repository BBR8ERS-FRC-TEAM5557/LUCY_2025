package frc.robot.state.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final double ambiguityThreshold = 0.4;
    public static final double solveDistCutoff = 4.0; // meters
    public static final double targetLogTimeSecs = 0.1;
    public static final double fieldBorderMargin = 0.25; // meters
    public static final double zMargin = 0.75; // meters
    public static final double xyStdDevCoefficient = 0.05; // was 0.005 meters
    public static final double thetaStdDevCoefficient = 0.05; // radians?

    public static final double[] stdDevFactors = new double[] { 1.0, 1.0 };

    public static final Pose3d[] robotToCameraPoses = new Pose3d[] {
            // Camera 1 - left
            new Pose3d(
                    new Translation3d(
                            Units.inchesToMeters(6.875),
                            Units.inchesToMeters(8.103),
                            Units.inchesToMeters(8.46)),
                    new Rotation3d(
                            0.0,
                            61.875,
                            35.0)),
            // Camera 2 - right
            new Pose3d(
                    new Translation3d(
                            Units.inchesToMeters(6.875),
                            -Units.inchesToMeters(8.103),
                            Units.inchesToMeters(8.46)),
                    new Rotation3d(
                            0.0,
                            61.875,
                            -35.0)),
    };

    public static final Pose3d[] cameraToRobotPoses = new Pose3d[] {
            // Camera 1 - left
            new Pose3d(
                    new Translation3d(
                            -Units.inchesToMeters(6.875),
                            -Units.inchesToMeters(8.103),
                            -Units.inchesToMeters(8.46)),
                    new Rotation3d(
                            -0.0,
                            -61.875,
                            -35.0)),
            // Camera 2 - right
            new Pose3d(
                    new Translation3d(
                            -Units.inchesToMeters(6.875),
                            Units.inchesToMeters(8.103),
                            -Units.inchesToMeters(8.46)),
                    new Rotation3d(
                            -0.0,
                            -61.875,
                            35.0)),
    };

    public static final String[] instanceNames = { "left", "right" };
}
