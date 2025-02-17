package frc.robot.state.vision;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.state.RobotStateEstimator;
import limelight.Limelight;
import limelight.estimator.LimelightPoseEstimator.BotPose;
import limelight.estimator.PoseEstimate;
import limelight.structures.LimelightSettings.LEDMode;
import limelight.structures.Orientation3d;

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
        camera.getSettings().withRobotOrientation(orientation);

        Optional<PoseEstimate> result = DriverStation.isEnabled() ? BotPose.BLUE_MEGATAG2.get(camera)
                : BotPose.BLUE.get(camera);

        if (result.isPresent()) {
            // If the pose cache timestamp was set, and the result is from the same
            // timestamp, give up
            if (poseCacheTimestampSeconds > 0
                    && Math.abs(poseCacheTimestampSeconds - result.get().timestampSeconds) < 1e-6) {
                inputs.tagsSeen = new int[] {};
                return;
            }

            inputs.estimatedRobotPose = result.get().pose;
            inputs.estimatedRobotPoseTimestamp = result.get().timestampSeconds;
            inputs.latency = result.get().latency;

            inputs.tagsSeen = new int[result.get().rawFiducials.length];
            for (int i = 0; i < inputs.tagsSeen.length; i++) {
                inputs.tagsSeen[i] = result.get().rawFiducials[i].id;
            }
            inputs.isMegatagTwo = result.get().isMegaTag2;

            poseCacheTimestampSeconds = result.get().timestampSeconds;

        }

        if (Timer.getFPGATimestamp() - poseCacheTimestampSeconds > 2.0) {
            disconnectedAlert.set(true);
        } else {
            disconnectedAlert.set(false);
        }
    }
}
