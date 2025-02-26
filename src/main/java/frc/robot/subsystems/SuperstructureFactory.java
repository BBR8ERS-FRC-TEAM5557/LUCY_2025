package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.Util;

public class SuperstructureFactory {
    private static final Elevator elevator = RobotContainer.m_elevator;
    private static final Wrist wrist = RobotContainer.m_wrist;

    private static Command pendingRumbleCommand = null;
    private static int level = 1;
    private static final Map<Integer, SuperstructureState> levelMap = new HashMap<>();
    static {
        levelMap.put(1, SuperstructureState.L1_CORAL);
        levelMap.put(2, SuperstructureState.L2_CORAL);
        levelMap.put(3, SuperstructureState.L3_CORAL);
        levelMap.put(4, SuperstructureState.L4_CORAL);
    }

    public static Command runSuperstructureState(SuperstructureState state) {
        return runSuperstructureState(() -> state);
    }

    public static Command runSuperstructureState(Supplier<SuperstructureState> state) {
        return Commands.parallel(
                elevator.runPositionCommand(state.get().getElevatorMetersSupplier()),
                wrist.runPositionCommand(state.get().getWristDegreesSupplier()));
    }

    public static Command waitUntilAtSetpoint() {
        return Commands.waitUntil(() -> elevator.atSetpoint() && wrist.atSetpoint());
    }

    public static Command scoreCoral() {
        return runSuperstructureState(() -> levelMap.get(level));
    }

    public static Command scoreL1Coral() {
        return runSuperstructureState(SuperstructureState.L1_CORAL);
    }

    public static Command scoreL2Coral() {
        return runSuperstructureState(SuperstructureState.L2_CORAL);
    }

    public static Command scoreL3Coral() {
        return runSuperstructureState(SuperstructureState.L3_CORAL);
    }

    public static Command scoreL4Coral() {
        return runSuperstructureState(SuperstructureState.L4_CORAL);
    }

    public static Command intakeCoral() {
        return runSuperstructureState(SuperstructureState.INTAKE_CORAL);
    }

    public static Command stow() {
        return runSuperstructureState(SuperstructureState.STOW);
    }

    public static Command adjustLevelSimple(int amount) {
        return Commands.runOnce(() -> {
            // Update the level
            level = (int) Util.clamp(level + amount, 1, 4);
            System.out.println("Level adjusted to: " + level);
        });
    }

    public static Command adjustLevel(int amount) {
        return Commands.runOnce(() -> {
            // Update the level
            level = (int) Util.clamp(level + amount, 1, 4);
            System.out.println("Level adjusted to: " + level);

            // Cancel the previous rumble command if it exists
            if (pendingRumbleCommand != null) {
                pendingRumbleCommand.cancel();
            }

            // Create a new rumble command
            Command rumbleCommand = Commands.waitSeconds(0.75) // Wait for 0.75 seconds
                    .andThen(
                            RobotContainer.controllerRumbleCommand()
                                    .withTimeout(0.2) // Buzz for 0.2 seconds
                                    .andThen(Commands.waitSeconds(0.1)) // Wait 0.1 seconds between buzzes
                                    .repeatedly() // Repeat for `level` times
                                    .withTimeout(0.3 * level))
                    .ignoringDisable(true); // Ensure this works even when the robot is disabled

            // Schedule the new rumble command
            pendingRumbleCommand = rumbleCommand;
            rumbleCommand.schedule();
        });
    }

    @AutoLogOutput(key = "Superstrucutre/selectedScoringLevel")
    public static int getLevel() {
        return level;
    }
}
