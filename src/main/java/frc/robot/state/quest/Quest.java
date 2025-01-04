package frc.robot.state.quest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.TimestampedFloat;
import edu.wpi.first.networktables.TimestampedFloatArray;
import frc.robot.util.Util;

import static edu.wpi.first.units.Units.Rotation;

import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.URL;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;

import limelight.Limelight;
import limelight.estimator.LimelightPoseEstimator;
import limelight.structures.LimelightData;
import limelight.structures.LimelightResults;
import limelight.structures.LimelightSettings;

/**
 * Limelight Camera class.
 */
public class Quest {

    /**
     * {@link Limelight} name.
     */
    public final String questName;

    // Configure Network Tables topics (oculus/...) to communicate with the Quest
    // HMD
    NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
    NetworkTable nt4Table = nt4Instance.getTable("oculus");
    private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
    private IntegerPublisher questMosi = nt4Table.getIntegerTopic("mosi").publish();

    // Subscribe to the Network Tables oculus data topics
    private DoubleSubscriber questBattery = nt4Table.getDoubleTopic("batteryLevel").subscribe(0.0f);
    private IntegerSubscriber questFrameCount = nt4Table.getIntegerTopic("frameCount").subscribe(0);
    private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
    private FloatArraySubscriber questPosition = nt4Table.getFloatArrayTopic("position")
            .subscribe(new float[] { 0.0f, 0.0f, 0.0f });
    private FloatArraySubscriber questQuaternion = nt4Table.getFloatArrayTopic("quaternion")
            .subscribe(new float[] { 0.0f, 0.0f, 0.0f, 0.0f });
    private FloatArraySubscriber questEulerAngles = nt4Table.getFloatArrayTopic("eulerAngles")
            .subscribe(new float[] { 0.0f, 0.0f, 0.0f });

    private long lastUpdateTime = 0;

    /**
     * Constructs and configures the {@link Limelight} NT Values.
     *
     * @param name Name of the limelight.
     */
    public Quest(String name) {
        questName = name;
    }

    /**
     * Gets the latest JSON {@link LimelightResults} output and returns a
     * LimelightResults object.
     *
     * @return LimelightResults object containing all current target data
     */
    public Optional<Pose2d> getPose() {
        Optional<Translation2d> position = getOculusPosition();
        Optional<Rotation2d> rotation = getOculusRotation();

        if (position.isEmpty() || rotation.isEmpty()) {
            return Optional.empty();
        }
        Pose2d pose = new Pose2d(position.get(), rotation.get());
        return Optional.of(pose);
    }

    public double getTimestamp() {
        return questTimestamp.get();
    }

    private Optional<Translation2d> getOculusPosition() {
        float[] update = questPosition.get();

        // if (Util.epsilonEquals(questPosition.getLastChange(), lastUpdateTime, 4)) {
        // lastUpdateTime = questPosition.getLastChange();
        // return Optional.empty();
        // }
        // lastUpdateTime = questPosition.getLastChange();

        Translation2d position = new Translation2d(update[2], -update[0]);
        return Optional.of(position);
    }

    private Optional<Rotation2d> getOculusRotation() {
        float[] update = questEulerAngles.get();

        // if (Util.epsilonEquals(questEulerAngles.getLastChange(), lastUpdateTime, 4))
        // {
        // lastUpdateTime = questEulerAngles.getLastChange();
        // return Optional.empty();
        // }
        // lastUpdateTime = questEulerAngles.getLastChange();

        Rotation2d rotation = Rotation2d.fromDegrees(update[1]);
        return Optional.of(rotation);

    }

    public double getOculusBattery() {
        return questBattery.get();
    }

    public boolean isConnected() {
        return nt4Instance.isConnected();
    }

    /**
     * Flush the NetworkTable data to server.
     */
    public void flush() {
        NetworkTableInstance.getDefault().flush();
    }

    /**
     * Get the {@link NetworkTable} for this limelight.
     *
     * @return {@link NetworkTable} for this limelight.
     */
    public NetworkTable getNTTable() {
        return NetworkTableInstance.getDefault().getTable(questName);
    }
}
