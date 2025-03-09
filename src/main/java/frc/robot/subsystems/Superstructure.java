package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import frc.lib.team6328.LoggedTunableNumber;

public class Superstructure {
        private static final LoggedTunableNumber home_elevator = new LoggedTunableNumber(
                        "Superstructure/HomeElevator",
                        0.0);
        private static final LoggedTunableNumber home_wrist = new LoggedTunableNumber(
                        "Superstructure/HomeWrist",
                        -55.0);
        private static final LoggedTunableNumber home_climb = new LoggedTunableNumber(
                        "Superstructure/HomeClimb",
                        -52.0);

        private static final LoggedTunableNumber stow_elevator = new LoggedTunableNumber(
                        "Superstructure/StartElevator",
                        0.0);
        private static final LoggedTunableNumber stow_wrist = new LoggedTunableNumber(
                        "Superstructure/StartWrist",
                        -55.0);
        private static final LoggedTunableNumber stow_climb = new LoggedTunableNumber(
                        "Superstructure/StartWrist",
                        -55.0);

        private static final LoggedTunableNumber intake_coral_elevator = new LoggedTunableNumber(
                        "Superstructure/IntakeCoralElevator",
                        0.26);
        private static final LoggedTunableNumber intake_coral_wrist = new LoggedTunableNumber(
                        "Superstructure/IntakeCoralWrist",
                        -45.0);
        private static final LoggedTunableNumber l2_prep_pop_algae_elevator = new LoggedTunableNumber(
                        "Superstructure/L2PrepPopAlgaeElevator",
                        0.228);
        private static final LoggedTunableNumber l2_prep_pop_algae_wrist = new LoggedTunableNumber(
                        "Superstructure/L2PrepPopAlgaeWrist",
                        150.0);
        private static final LoggedTunableNumber l2_pop_algae_elevator = new LoggedTunableNumber(
                        "Superstructure/L2PopAlgaeElevator",
                        0.3);
        private static final LoggedTunableNumber l2_pop_algae_wrist = new LoggedTunableNumber(
                        "Superstructure/L2PopAlgaeWrist",
                        147.0);
        private static final LoggedTunableNumber l3_prep_pop_algae_elevator = new LoggedTunableNumber(
                        "Superstructure/L3PrepPopAlgaeElevator",
                        0.4);
        private static final LoggedTunableNumber l3_prep_pop_algae_wrist = new LoggedTunableNumber(
                        "Superstructure/L3PrepPopAlgaeWrist",
                        150.0);
        private static final LoggedTunableNumber l3_pop_algae_elevator = new LoggedTunableNumber(
                        "Superstructure/L3PopAlgaeElevator",
                        0.5);
        private static final LoggedTunableNumber l3_pop_algae_wrist = new LoggedTunableNumber(
                        "Superstructure/L3PopAlgaeWrist",
                        120.0);

        private static final LoggedTunableNumber l1_coral_elevator = new LoggedTunableNumber(
                        "Superstructure/L1CoralElevator",
                        0.0);
        private static final LoggedTunableNumber l1_coral_wrist = new LoggedTunableNumber(
                        "Superstructure/L1CoralWrist",
                        -50.0);
        private static final LoggedTunableNumber l2_coral_elevator = new LoggedTunableNumber(
                        "Superstructure/L2CoralElevator",
                        0.0);
        private static final LoggedTunableNumber l2_coral_wrist = new LoggedTunableNumber(
                        "Superstructure/L2CoralWrist",
                        150.0);

        private static final LoggedTunableNumber l3_coral_elevator = new LoggedTunableNumber(
                        "Superstructure/L3CoralElevator",
                        0.27);
        private static final LoggedTunableNumber l3_coral_wrist = new LoggedTunableNumber(
                        "Superstructure/L3CoralWrist",
                        135.0);
        private static final LoggedTunableNumber l4_coral_elevator = new LoggedTunableNumber(
                        "Superstructure/L4CoralElevator",
                        1.180);
        private static final LoggedTunableNumber l4_coral_wrist = new LoggedTunableNumber(
                        "Superstructure/L4CoralWrist",
                        163.0);

        private static final LoggedTunableNumber prep_climb_wrist = new LoggedTunableNumber(
                        "Superstructure/PrepClimbWrist",
                        200.0);
        private static final LoggedTunableNumber prep_climb_climb = new LoggedTunableNumber(
                        "Superstructure/PrepClimbClimb",
                        147.0);
        private static final LoggedTunableNumber deep_climb_climb = new LoggedTunableNumber(
                        "Superstructure/ExecuteDeepClimbClimb",
                        40.0);

        private static final LoggedTunableNumber l4_coral_wrist_transition = new LoggedTunableNumber(
                        "Superstructure/ExecuteDeepClimbClimb",
                        40.0);

        public enum SuperstructureState {
                HOME(() -> home_elevator.get(), () -> home_wrist.get(), () -> home_climb.get()),
                STOW(() -> stow_elevator.get(), () -> stow_wrist.get(), () -> stow_climb.get()),

                INTAKE_CORAL(() -> intake_coral_elevator.get(), () -> intake_coral_wrist.get(), () -> stow_climb.get()),

                L1_CORAL(() -> l1_coral_elevator.get(), () -> l1_coral_wrist.get(), () -> stow_climb.get()),
                L2_CORAL(() -> l2_coral_elevator.get(), () -> l2_coral_wrist.get(), () -> stow_climb.get()),
                L3_CORAL(() -> l3_coral_elevator.get(), () -> l3_coral_wrist.get(), () -> stow_climb.get()),
                L4_CORAL(() -> l4_coral_elevator.get(), () -> l4_coral_wrist.get(), () -> stow_climb.get()),
                L4_CORAL_TRANSITION(() -> l4_coral_elevator.get(), () -> l4_coral_wrist_transition.get(), () -> stow_climb.get()),

                L2_PREP_POP_ALGAE(() -> l2_prep_pop_algae_elevator.get(), () -> l2_prep_pop_algae_wrist.get(), () -> stow_climb.get()),
                L2_POP_ALGAE(() -> l2_pop_algae_elevator.get(), () -> l2_pop_algae_wrist.get(), () -> stow_climb.get()),
                L3_PREP_POP_ALGAE(() -> l3_prep_pop_algae_elevator.get(), () -> l3_prep_pop_algae_wrist.get(), () -> stow_climb.get()),
                L3_POP_ALGAE(() -> l3_pop_algae_elevator.get(), () -> l3_pop_algae_wrist.get(), () -> stow_climb.get()),

                PREP_DEEP_CLIMB(() -> stow_elevator.get(), () -> prep_climb_wrist.get(), () -> prep_climb_climb.get()),
                DEEP_CLIMB(() -> stow_elevator.get(), () -> prep_climb_wrist.get(), () -> deep_climb_climb.get());

                private final DoubleSupplier elevatorSetpoint;
                private final DoubleSupplier wristSetpoint;
                private final DoubleSupplier climbSetpoint;

                private SuperstructureState(DoubleSupplier elevatorSetpoint, DoubleSupplier wristSetpoint, DoubleSupplier climbSetpoint) {
                        this.elevatorSetpoint = elevatorSetpoint;
                        this.wristSetpoint = wristSetpoint;
                        this.climbSetpoint = climbSetpoint;
                }

                public double getElevatorMeters() {
                        return elevatorSetpoint.getAsDouble();
                }

                public DoubleSupplier getElevatorMetersSupplier() {
                        return elevatorSetpoint;
                }

                public double getWristDegrees() {
                        return wristSetpoint.getAsDouble();
                }

                public DoubleSupplier getWristDegreesSupplier() {
                        return wristSetpoint;
                }

                public double getClimbDegrees() {
                        return climbSetpoint.getAsDouble();
                }

                public DoubleSupplier getClimbDegreesSupplier() {
                        return climbSetpoint;
                }
        }
}
