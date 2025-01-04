package frc.robot.state.vision;

import static frc.robot.state.vision.VisionConstants.*;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.lib.team6328.VirtualSubsystem;
import frc.robot.state.RobotStateEstimator;
import frc.robot.state.vision.AprilTagVisionIO.AprilTagVisionIOInputs;
import frc.robot.util.FieldConstants;
import frc.robot.util.GeometryUtil;

public class Vision extends VirtualSubsystem {
    public record VisionObservation(Pose2d visionPose, double timestamp, Matrix<N3, N1> stdDevs) {
    }

    private final AprilTagVisionIO[] io;
    private final AprilTagVisionIOInputs[] inputs;

    private final Map<Integer, Double> lastFrameTimes = new HashMap<>();
    private final Map<Integer, Double> lastTagDetectionTimes = new HashMap<>();

    public Vision(AprilTagVisionIO... io) {
        System.out.println("[Init] Creating Vision");

        this.io = io;
        inputs = new AprilTagVisionIOInputs[io.length];
        for (int i = 0; i < io.length; i++) {
            inputs[i] = new AprilTagVisionIOInputs();
        }

        // Create map of last frame times for instances
        for (int i = 0; i < io.length; i++) {
            lastFrameTimes.put(i, 0.0);
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < io.length; i++) {
            io[i].updateInputs(inputs[i]);
            Logger.processInputs("AprilTagVision/" + instanceNames[i], inputs[i]);
        }

        // Loop over instances
        List<Pose3d> allRobotPoses3d = new ArrayList<>();
        List<VisionObservation> allVisionObservations = new ArrayList<>();

        for (int instanceIndex = 0; instanceIndex < io.length; instanceIndex++) {
            boolean hasUpdate = inputs[instanceIndex].estimatedRobotPoseTimestamp
                    - lastFrameTimes.get(instanceIndex) > 1e-5;

            // Exit if no new data
            if (!hasUpdate) {
                Logger.recordOutput("AprilTagVision/Inst" + instanceNames[instanceIndex] + "/RobotPose", new Pose2d());
                Logger.recordOutput("AprilTagVision/Inst" + instanceNames[instanceIndex] + "/RobotPose3d",
                        new Pose3d());

                // If no recent frames from instance, clear tag poses
                if (Timer.getFPGATimestamp() - lastFrameTimes.get(instanceIndex) > targetLogTimeSecs) {
                    Logger.recordOutput("AprilTagVision/" + instanceNames[instanceIndex] + "/TagPoses",
                            new Pose3d[] {});
                }
                continue;
            }

            lastFrameTimes.put(instanceIndex, Timer.getFPGATimestamp());
            var timestamp = inputs[instanceIndex].estimatedRobotPoseTimestamp;
            Pose3d robotPose3d = inputs[instanceIndex].estimatedRobotPose;
            Pose3d cameraPose3d = robotPose3d
                    .transformBy(GeometryUtil.pose3dToTransform3d(robotToCameraPoses[instanceIndex]));

            // Exit if robot pose is off the field
            if (robotPose3d.getX() < -fieldBorderMargin
                    || robotPose3d.getX() > FieldConstants.fieldLength + fieldBorderMargin
                    || robotPose3d.getY() < -fieldBorderMargin
                    || robotPose3d.getY() > FieldConstants.fieldWidth + fieldBorderMargin
                    || robotPose3d.getZ() < -zMargin
                    || robotPose3d.getZ() > zMargin) {
                continue;
            }

            // Get tag poses and update last detection times
            List<Pose3d> tagPoses = new ArrayList<>();
            for (int i = 0; i < inputs[instanceIndex].tagsSeen.length; i++) {
                int tagId = inputs[instanceIndex].tagsSeen[i];
                lastTagDetectionTimes.put(tagId, Timer.getFPGATimestamp());
                Optional<Pose3d> tagPose = FieldConstants.aprilTags.getTagPose(tagId);
                tagPose.ifPresent(tagPoses::add);
            }
            if (tagPoses.size() == 0) {
                continue;
            }

            // Calculate average distance to tag
            double totalDistance = 0.0;
            for (Pose3d tagPose : tagPoses) {
                totalDistance += tagPose.getTranslation().getDistance(cameraPose3d.getTranslation());
            }
            double avgDistance = totalDistance / tagPoses.size();

            // calculate standard deviations
            double xyStdDev = xyStdDevCoefficient
                    * Math.pow(avgDistance, 2.0)
                    / tagPoses.size()
                    * stdDevFactors[instanceIndex];

            // only use vision to adjust robot rotation if pose was calculated with megatag
            // 1
            boolean useVisionRotation = !inputs[instanceIndex].isMegatagTwo;
            double thetaStdDev = useVisionRotation
                    ? thetaStdDevCoefficient
                            * Math.pow(avgDistance, 2.0)
                            / tagPoses.size()
                            * stdDevFactors[instanceIndex]
                    : Double.POSITIVE_INFINITY;

            // add information to lists
            allVisionObservations.add(
                    new VisionObservation(
                            robotPose3d.toPose2d(), timestamp, VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev)));
            allRobotPoses3d.add(robotPose3d);

            // Log data from instance
            Logger.recordOutput(
                    "AprilTagVision/" + instanceNames[instanceIndex] + "/LatencySecs",
                    Timer.getFPGATimestamp() - timestamp);
            Logger.recordOutput("AprilTagVision/" + instanceNames[instanceIndex] + "/RobotPose3d", robotPose3d);
            Logger.recordOutput(
                    "AprilTagVision/" + instanceNames[instanceIndex] + "/TagPoses", tagPoses.toArray(Pose3d[]::new));

        }

        // Log robot poses
        Logger.recordOutput("AprilTagVision/RobotPoses3d", allRobotPoses3d.toArray(Pose3d[]::new));

        // Send results to robot state estimator
        allVisionObservations.stream().sorted(Comparator.comparingDouble(VisionObservation::timestamp))
                .forEach(RobotStateEstimator.getInstance()::addVisionObservation);
    }
}
