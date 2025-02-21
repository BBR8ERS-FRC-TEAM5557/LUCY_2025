package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.wpilibj.Timer;
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

    private static Timer levelAdjustmentTimer;
    private static int level = 1;
    private static final Map<Integer, SuperstructureState> levelMap = new HashMap<>();
    static {
        levelMap.put(1, SuperstructureState.L1_CORAL);
        levelMap.put(2, SuperstructureState.L2_CORAL);
        levelMap.put(3, SuperstructureState.L3_CORAL);
        levelMap.put(4, SuperstructureState.L4_CORAL);
    }

    public static Command runSuperstructureState(SuperstructureState state) {
        return Commands.parallel(
                elevator.runPositionCommand(state.getElevatorMetersSupplier()),
                wrist.runPositionCommand(state.getWristDegreesSupplier()));
                
    }

    public static Command scoreCoral() {
        return runSuperstructureState(levelMap.get(level));
    }

    public static Command intakeCoral() {
        return runSuperstructureState(SuperstructureState.INTAKE_CORAL);
    }

    public static Command stow() {
        return runSuperstructureState(SuperstructureState.STOW);
    }

    public static Command adjustLevel(int amount) {
        return Commands.startRun(
                () -> {
                    levelAdjustmentTimer = new Timer();
                    levelAdjustmentTimer.reset();
                    levelAdjustmentTimer.start();
                    level = (int) Util.clamp(level + amount, 1, 4);
                },
                () -> {
                })
                .until(() -> levelAdjustmentTimer.hasElapsed(0.75))
                .andThen(
                        RobotContainer.controllerRumbleCommand()
                                .withTimeout(0.2)
                                .andThen(Commands.waitSeconds(0.1))
                                .repeatedly()
                                .withTimeout(0.3 * level));

    }

    public static int getLevel() {
        return level;
    }
}
