package frc.robot.state.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import frc.robot.state.RobotStateEstimator;
import limelight.Limelight;
import limelight.networktables.LimelightPoseEstimator.BotPose;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.networktables.Orientation3d;
import limelight.networktables.PoseEstimate;

public class AprilTagVisionIOLimelight implements AprilTagVisionIO {
    private final Limelight camera;

    private double poseCacheTimestampSeconds = -1;
    private final Alert disconnectedAlert;

    /**
     * Creates a new VisionIOPhotonVision object.
     *
     * @param cameraName the name of the PhotonVision camera to use; the name must
     *                   be unique
     */
    public AprilTagVisionIOLimelight(String cameraName, Pose3d cameraToRobot) {
        System.out.println("[Init] Creating Limelight(" + cameraName + ")");

        this.camera = new Limelight(cameraName);
        this.disconnectedAlert = new Alert("No data from \"" + cameraName + "\"", Alert.AlertType.kInfo);

        camera.getSettings()
                .withLimelightLEDMode(LEDMode.PipelineControl)
                .withCameraOffset(cameraToRobot)
                .save();
    }

    /**
     * Updates the specified VisionIOInputs object with the latest data from the
     * camera.
     *
     * @param inputs the VisionIOInputs object to update with the latest data from
     *               the camera
     */
    @Override
    public void updateInputs(AprilTagVisionIOInputs inputs) {
        Rotation3d rotation = new Rotation3d(
                0.0, 0.0, RobotStateEstimator.getInstance().getEstimatedPose().getRotation().getRadians());
        Orientation3d orientation = new Orientation3d(rotation, null);
        camera.getSettings().withRobotOrientation(orientation).save();;

        Optional<PoseEstimate> megaTag1 = BotPose.BLUE.get(camera);
        Optional<PoseEstimate> megaTag2 = BotPose.BLUE_MEGATAG2.get(camera);

        double avgDistance = Double.MAX_VALUE;
        if(megaTag2.isPresent()) {
            avgDistance = megaTag2.get().avgTagDist;
        } else if(megaTag1.isPresent()) {
            avgDistance = megaTag1.get().avgTagDist;
        }

        Optional<PoseEstimate> result = Optional.empty();
        double rotationalSpeed = Math.abs(RobotContainer.m_swerve.getCurrentFieldChassisSpeeds().omegaRadiansPerSecond);

        if(avgDistance > 1.5 && rotationalSpeed < Units.degreesToRadians(180) && megaTag2.isPresent()) {
            result = megaTag2;
        } else {
            result = megaTag1;
        }

        if (result.isPresent()) {
            inputs.estimatedRobotPose = result.get().pose;
            inputs.estimatedRobotPoseTimestamp = result.get().timestampSeconds;
            inputs.latencyMS = result.get().latency * 1000.0;
            inputs.tagsSeen = new int[result.get().rawFiducials.length];
            for (int i = 0; i < inputs.tagsSeen.length; i++) {
                inputs.tagsSeen[i] = result.get().rawFiducials[i].id;
            }
            inputs.isMegatagTwo = result.get().isMegaTag2;

            poseCacheTimestampSeconds = inputs.estimatedRobotPoseTimestamp;
        } else {
            inputs.tagsSeen = new int[] {};
        }

        if (Timer.getFPGATimestamp() - poseCacheTimestampSeconds > 2.0) {
            disconnectedAlert.set(true);
        } else {
            disconnectedAlert.set(false);
        }
    }
}
