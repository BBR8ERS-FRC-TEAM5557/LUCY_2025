package frc.robot.state.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class VslamVisionIOQuest implements VslamVisionIO {
    private final Quest quest;
    private final Pose3d robotToCamera;

    /**
     * Creates a new VisionIOPhotonVision object.
     *
     * @param cameraName the name of the quest camera to use; the name must
     *                   be unique
     */
    public VslamVisionIOQuest(String cameraName, Pose3d robotToCamera) {
        System.out.println("[Init] Creating Quest(" + cameraName + ")");

        this.quest = new Quest(cameraName);
        this.robotToCamera = robotToCamera;
    }

    /**
     * Updates the specified VisionIOInputs object with the latest data from the
     * camera.
     *
     * @param inputs the VisionIOInputs object to update with the latest data from
     *               the camera
     */
    @Override
    public void updateInputs(VslamVisionIOInputs inputs) {
        if (quest.isConnected()) {
            inputs.batteryLevel = quest.getOculusBattery();

            Pose2d fieldToCamera = quest.getPose().get();
            Translation2d fieldToRobotTransform = fieldToCamera.minus(robotToCamera.toPose2d()).getTranslation();
            Rotation2d fieldToRobotRotation = fieldToCamera.getRotation()
                    .minus(robotToCamera.getRotation().toRotation2d());
            inputs.estimatedRobotPose = new Pose2d(fieldToRobotTransform, fieldToRobotRotation);

            inputs.estimatedRobotPoseTimestamp = quest.getTimestamp();
        }
    }
}