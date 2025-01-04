package frc.robot.state.vision;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose3d;

public interface AprilTagVisionIO {

    @AutoLog
    public static class AprilTagVisionIOInputs implements LoggableInputs {
        Pose3d estimatedRobotPose = new Pose3d();
        double estimatedRobotPoseTimestamp = 0.0;
        double latency = 0.0;
        int[] tagsSeen = new int[] {};
        boolean isMegatagTwo = true;

        @Override
        public void toLog(LogTable table) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'toLog'");
        }

        @Override
        public void fromLog(LogTable table) {
            // TODO Auto-generated method stub
            throw new UnsupportedOperationException("Unimplemented method 'fromLog'");
        }
    }

    /**
     * Updates the set of loggable inputs.
     *
     * @param inputs the inputs to update
     */
    default void updateInputs(AprilTagVisionIOInputs inputs) {
    }
}
