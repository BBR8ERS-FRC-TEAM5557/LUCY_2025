package frc.robot.state.vision;

import org.littletonrobotics.junction.AutoLog;
import edu.wpi.first.math.geometry.Pose3d;

public interface AprilTagVisionIO {

    @AutoLog
    public static class AprilTagVisionIOInputs {
        Pose3d estimatedRobotPose = new Pose3d();
        double estimatedRobotPoseTimestamp = 0.0;
        double latency = 0.0;
        int[] tagsSeen = new int[] {};
        boolean isMegatagTwo = true;
    }

    /**
     * Updates the set of loggable inputs.
     *
     * @param inputs the inputs to update
     */
    default void updateInputs(AprilTagVisionIOInputs inputs) {
    }
}
