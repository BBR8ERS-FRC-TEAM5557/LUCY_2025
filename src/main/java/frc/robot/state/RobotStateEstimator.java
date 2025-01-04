package frc.robot.state;

import java.util.List;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.lib.team6328.VirtualSubsystem;
import frc.robot.RobotContainer;
import frc.robot.state.quest.Quest;
import frc.robot.state.vision.Vision.VisionObservation;

public class RobotStateEstimator extends VirtualSubsystem {
    public record QuestObservation(Pose3d questPose) {
    }

    private static RobotStateEstimator mInstance = null;

    public static RobotStateEstimator getInstance() {
        if (mInstance == null) {
            System.out.println("[Init] Creating RobotStateEstimator");
            mInstance = new RobotStateEstimator();
        }
        return mInstance;
    }

    private Twist2d mRobotVelocity = new Twist2d();
    private final Field2d mField2d = new Field2d();

    private RobotStateEstimator() {
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Driver");
        shuffleboardTab.add(mField2d);
    }

    @Override
    public void periodic() {
        clampPoseToField();
        updateFieldWidget();
    }

    public void addVisionObservation(VisionObservation update) {
        RobotContainer.m_swerve.addVisionMeasurement(update.visionPose(), update.timestamp(), update.stdDevs());
    }

    public void addVisionObservation(List<VisionObservation> visionData) {
        for (var update : visionData) {
            addVisionObservation(update);
        }
    }

    public void addQuestObservation(QuestObservation update) {

    }

    public void addQuestObservation(List<QuestObservation> questData) {
        for (var update : questData) {
            addQuestObservation(update);
        }
    }

    @AutoLogOutput(key = "RobotState/FusedFieldVelocity")
    public Twist2d getFusedFieldVelocity() {
        return RobotContainer.m_swerve.getCurrentFieldChassisSpeeds();
    }

    @AutoLogOutput(key = "RobotState/EstimatedPose")
    public Pose2d getEstimatedPose() {
        return RobotContainer.m_swerve.getState().Pose;
    }

    /**
     * Predicts what our pose will be in the future. Allows separate translation and
     * rotation
     * lookaheads to account for varying latencies in the different measurements.
     *
     * @param translationLookaheadS The lookahead time for the translation of the
     *                              robot
     * @param rotationLookaheadS    The lookahead time for the rotation of the robot
     * @return The predicted pose.
     */
    public Pose2d getPredictedPose(double translationLookaheadS, double rotationLookaheadS) {
        return getEstimatedPose()
                .exp(
                        new Twist2d(
                                mRobotVelocity.dx * translationLookaheadS,
                                mRobotVelocity.dy * translationLookaheadS,
                                mRobotVelocity.dtheta * rotationLookaheadS));
    }

    public void setPose(Pose2d pose) {
        RobotContainer.m_swerve.seedFieldRelative(pose);
    }

    private void clampPoseToField() {
        // if out of bounds, clamp to field
        double estimatedXPos = getEstimatedPose().getX();
        double estimatedYPos = getEstimatedPose().getY();
        if (estimatedYPos < 0.0
                || estimatedYPos > 8.35
                || estimatedXPos < 0.0
                || estimatedXPos > Units.feetToMeters(52)) {
            double clampedYPosition = MathUtil.clamp(estimatedYPos, 0.0, 8.35);
            double clampedXPosition = MathUtil.clamp(estimatedXPos, 0.0, Units.feetToMeters(52.0));
            this.setPose(new Pose2d(clampedXPosition, clampedYPosition, getEstimatedPose().getRotation()));
        }
    }

    private void updateFieldWidget() {
        Pose2d robotPose = getEstimatedPose();
        mField2d.setRobotPose(robotPose);
    }

    public void addFieldPose(String name, Pose2d pose) {
        if (pose != null) {
            mField2d.getObject(name).setPose(pose);
        }
    }

    public void addFieldTrajectory(String name, Trajectory traj) {
        if (traj != null) {
            mField2d.getObject(name).setTrajectory(traj);
        }
    }
}
