package frc.robot.state.quest;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

public interface VslamVisionIO {
    @AutoLog
    public static class VslamVisionIOInputs implements LoggableInputs {
        Pose2d estimatedRobotPose = new Pose2d();
        double estimatedRobotPoseTimestamp = 0.0;
        double fps = 0.0;
        double batteryLevel = 0.0;

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
    default void updateInputs(VslamVisionIOInputs inputs) {
    }
}