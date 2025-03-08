package frc.robot.subsystems;

import java.util.Map;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.wrist.Wrist;
import frc.robot.util.Util;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SuperstructureFactory {
        private static final Elevator elevator = RobotContainer.m_elevator;
        private static final Wrist wrist = RobotContainer.m_wrist;
        private static final Climb climb = RobotContainer.m_climb;

        private static Command pendingRumbleCommand = null;
        private static int level = 4;

        public static Command runSuperstructureState(SuperstructureState state) {
                return runSuperstructureState(() -> state);
        }

        public static Command runSuperstructureState(Supplier<SuperstructureState> state) {
                return Commands.parallel(
                                elevator.runPositionCommand(state.get().getElevatorMetersSupplier()),
                                wrist.runPositionCommand(state.get().getWristDegreesSupplier()),
                                climb.runPositionCommand(state.get().getClimbDegreesSupplier()));

                // return Commands.parallel(
                // elevator.runPositionCommand(state.get().getElevatorMetersSupplier()),
                // elevator.waitUntilAboveCommand(
                // () -> state.get().getElevatorMeters()
                // - elevator_tolerance_before_moving_wrist.get())
                // .andThen(wrist.runPositionCommand(state.get()
                // .getWristDegreesSupplier())));
        }

        public static Command waitUntilAtSetpoint() {
                return Commands.waitUntil(() -> elevator.atSetpoint() && wrist.atSetpoint() && climb.atSetpoint());
        }

        public static Command scoreCoral() {
                return Commands.select(
                                Map.of(
                                                1, scoreL1Coral(),
                                                2, scoreL2Coral(),
                                                3, scoreL3Coral(),
                                                4, scoreL4Coral()),
                                () -> getLevel());
        }

        public static Command prepPopAlgae() {
                return Commands.select(
                                Map.of(
                                                1, stow(),
                                                2, l2PrepPopAlgae(),
                                                3, l3PrepPopAlgae(),
                                                4, stow()),
                                () -> getLevel());
        }

        public static Command executePopAlgae() {
                return Commands.select(
                                Map.of(
                                                1, stow(),
                                                2, l2PopAlgae(),
                                                3, l3PopAlgae(),
                                                4, stow()),
                                () -> getLevel());
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

        public static Command l2PrepPopAlgae() {
                return runSuperstructureState(SuperstructureState.L2_PREP_POP_ALGAE);
        }

        public static Command l2PopAlgae() {
                return runSuperstructureState(SuperstructureState.L2_POP_ALGAE);
        }

        public static Command l3PrepPopAlgae() {
                return runSuperstructureState(SuperstructureState.L3_PREP_POP_ALGAE);
        }

        public static Command l3PopAlgae() {
                return runSuperstructureState(SuperstructureState.L3_POP_ALGAE);
        }

        public static Command stow() {
                return runSuperstructureState(SuperstructureState.STOW);
        }

        public static Command prepDeepClimb() {
                return runSuperstructureState(SuperstructureState.PREP_DEEP_CLIMB);
        }

        public static Command deepClimb() {
                return runSuperstructureState(SuperstructureState.DEEP_CLIMB);
        }

        public static Command adjustLevel(int amount, double rumbleWait) {
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
                                                                        .andThen(Commands.waitSeconds(0.1)) // Wait 0.1
                                                                                                            // seconds
                                                                                                            // between
                                                                                                            // buzzes
                                                                        .repeatedly() // Repeat for `level` times
                                                                        .withTimeout(0.3 * level))
                                        .ignoringDisable(true); // Ensure this works even when the robot is disabled

                        // Schedule the new rumble command
                        pendingRumbleCommand = rumbleCommand;
                        rumbleCommand.schedule();
                        SmartDashboard.putNumber("Level", getLevel());
                });
        }

        @AutoLogOutput(key = "Superstructure/selectedScoringLevel")
        public static int getLevel() {
                return level;
        }
}
