package frc.robot.subsystems.vision;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.VirtualSubsystem;
import frc.robot.Constants.VisionConstants;

public class VisionSubsystem extends VirtualSubsystem {
    private final PhotonCamera m_camera;
    private final PhotonPoseEstimator m_poseEstimator;
    private final AprilTagFieldLayout m_fieldLayout;

    // Vision measurement data
    private Optional<EstimatedRobotPose> m_estimatedPose = Optional.empty();
    private double m_lastTimestamp = 0.0;
    private boolean m_hasTargets = false;
    private double m_bestTargetYaw = 0.0;
    private double m_bestTargetPitch = 0.0;
    private double m_bestTargetArea = 0.0;
    private int m_bestTargetId = -1;

    public VisionSubsystem() {
        m_camera = new PhotonCamera(VisionConstants.kLimelightName);

        // Load AprilTag field layout
        m_fieldLayout = VisionConstants.kAprilTagField.loadAprilTagLayoutField();

        // Create pose estimator
        m_poseEstimator = new PhotonPoseEstimator(
            m_fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            new Transform3d(
                new Translation3d(
                    VisionConstants.kLimelightMountOffsetX,
                    VisionConstants.kLimelightMountOffsetY,
                    VisionConstants.kLimelightLensHeightMeters),
                new Rotation3d(0, Math.toRadians(-VisionConstants.kLimelightMountAngleDegrees), 0)
            )
        );

        m_poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    }

    @Override
    public void periodic() {
        // Update vision data
        var result = m_camera.getLatestResult();
        m_hasTargets = result.hasTargets();

        if (m_hasTargets) {
            var bestTarget = result.getBestTarget();
            m_bestTargetYaw = bestTarget.getYaw();
            m_bestTargetPitch = bestTarget.getPitch();
            m_bestTargetArea = bestTarget.getArea();
            m_bestTargetId = bestTarget.getFiducialId();

            // Update pose estimation
            m_estimatedPose = m_poseEstimator.update(result);
            if (m_estimatedPose.isPresent()) {
                m_lastTimestamp = m_estimatedPose.get().timestampSeconds;
            }
        } else {
            m_bestTargetYaw = 0.0;
            m_bestTargetPitch = 0.0;
            m_bestTargetArea = 0.0;
            m_bestTargetId = -1;
            m_estimatedPose = Optional.empty();
        }

        // Log vision data
        Logger.recordOutput("Vision/HasTargets", m_hasTargets);
        Logger.recordOutput("Vision/BestTargetYaw", m_bestTargetYaw);
        Logger.recordOutput("Vision/BestTargetPitch", m_bestTargetPitch);
        Logger.recordOutput("Vision/BestTargetArea", m_bestTargetArea);
        Logger.recordOutput("Vision/BestTargetID", m_bestTargetId);

        if (m_estimatedPose.isPresent()) {
            Logger.recordOutput("Vision/EstimatedPose",
                m_estimatedPose.get().estimatedPose.toPose2d());
            Logger.recordOutput("Vision/PoseTimestamp", m_lastTimestamp);
        }
    }

    /**
     * Gets the latest estimated robot pose from vision
     * @return Optional containing the estimated pose and timestamp
     */
    public Optional<EstimatedRobotPose> getEstimatedPose() {
        return m_estimatedPose;
    }

    /**
     * Gets the pose estimate as a Pose2d for easier use
     * @return Optional containing the 2D pose estimate
     */
    public Optional<Pose2d> getEstimatedPose2d() {
        return m_estimatedPose.map(pose -> pose.estimatedPose.toPose2d());
    }

    /**
     * Gets the timestamp of the last vision measurement
     * @return timestamp in seconds
     */
    public double getLastTimestamp() {
        return m_lastTimestamp;
    }

    /**
     * Checks if the vision system currently sees any targets
     * @return true if targets are detected
     */
    public boolean hasTargets() {
        return m_hasTargets;
    }

    /**
     * Gets the yaw angle to the best target (horizontal offset)
     * @return yaw angle in degrees, positive is to the right
     */
    public double getBestTargetYaw() {
        return m_bestTargetYaw;
    }

    /**
     * Gets the pitch angle to the best target (vertical offset)
     * @return pitch angle in degrees, positive is up
     */
    public double getBestTargetPitch() {
        return m_bestTargetPitch;
    }

    /**
     * Gets the area of the best target as a percentage of the image
     * @return target area (0-100)
     */
    public double getBestTargetArea() {
        return m_bestTargetArea;
    }

    /**
     * Gets the ID of the best target
     * @return AprilTag ID, or -1 if no target
     */
    public int getBestTargetId() {
        return m_bestTargetId;
    }

    /**
     * Gets the standard deviations for vision measurements
     * @param pose The vision pose measurement
     * @return Matrix of standard deviations for x, y, and rotation
     */
    public Matrix<N3, N1> getEstimationStdDevs(Pose2d pose) {
        var estStdDevs = VecBuilder.fill(
            VisionConstants.kVisionTranslationStdDev,
            VisionConstants.kVisionTranslationStdDev,
            VisionConstants.kVisionRotationStdDev);

        // Increase std devs if multiple targets are not in view
        var result = m_camera.getLatestResult();
        if (result.hasTargets()) {
            var targets = result.getTargets();
            int numTags = targets.size();
            double avgDist = targets.stream()
                .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
                .average().orElse(0);

            // Decrease std devs if multiple targets and close
            if (numTags == 0) {
                estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
            } else if (numTags >= 2) {
                estStdDevs = VecBuilder.fill(
                    VisionConstants.kVisionTranslationStdDev * 0.5,
                    VisionConstants.kVisionTranslationStdDev * 0.5,
                    VisionConstants.kVisionRotationStdDev * 0.5);
            } else if (avgDist < 4) {
                estStdDevs = VecBuilder.fill(
                    VisionConstants.kVisionTranslationStdDev * 0.8,
                    VisionConstants.kVisionTranslationStdDev * 0.8,
                    VisionConstants.kVisionRotationStdDev * 0.8);
            }
        } else {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }

        return estStdDevs;
    }
}