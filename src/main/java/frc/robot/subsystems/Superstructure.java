package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import frc.lib.team6328.LoggedTunableNumber;

public class Superstructure {

        /** HOME **/
        private static final LoggedTunableNumber home_elevator = new LoggedTunableNumber(
                        "Superstructure/HomeElevator",
                        0.0);
        private static final LoggedTunableNumber home_wrist = new LoggedTunableNumber(   //TODO: IS THERE EVEN A HARDSTOP? IF THERE ISN'T, DONT HOME
                        "Superstructure/HomeWrist",
                        90.0);
        private static final LoggedTunableNumber home_climb = new LoggedTunableNumber(
                        "Superstructure/HomeClimb",
                        0.0);

        private static final LoggedTunableNumber home_intake = new LoggedTunableNumber(
                        "Superstructure/HomeIntake",
                        85.0);

        /** STOW **/
        private static final LoggedTunableNumber stow_elevator = new LoggedTunableNumber(
                        "Superstructure/StowElevator",
                        0.5);
        private static final LoggedTunableNumber stow_wrist = new LoggedTunableNumber(
                        "Superstructure/StowWrist",
                        270.0);
        private static final LoggedTunableNumber stow_climb = new LoggedTunableNumber(
                        "Superstructure/StowWrist",
                        10.0);
        private static final LoggedTunableNumber stow_intake = new LoggedTunableNumber(
                        "Superstructure/StowIntake",
                        90.0);
        /** INTAKE **/
        private static final LoggedTunableNumber intake_coral_elevator = new LoggedTunableNumber(
                        "Superstructure/IntakeCoralElevator",
                        0.5);
        private static final LoggedTunableNumber intake_coral_wrist = new LoggedTunableNumber(
                        "Superstructure/IntakeCoralWrist",
                        270.0);
        private static final LoggedTunableNumber intake_coral_intake = new LoggedTunableNumber(
                        "Superstructure/IntakeCoralIntake",
                        195.0);    
        
        /** INTAKE HAND OFF **/
        private static final LoggedTunableNumber handoff_coral_elevator = new LoggedTunableNumber(
                        "Superstructure/HandoffCoralElevator",
                        0.5);        
        private static final LoggedTunableNumber handoff_coral_wrist = new LoggedTunableNumber(
                        "Superstructure/HandoffCoralWrist",
                        270.0);
        private static final LoggedTunableNumber handoff_coral_intake = new LoggedTunableNumber(
                                "Superstructure/HandoffCoralIntake",
                         85.0);                   
                        
        

        /** PREP POP ALGAE **/
                        //L2
        private static final LoggedTunableNumber l2_prep_pop_algae_elevator = new LoggedTunableNumber(
                        "Superstructure/L2PrepPopAlgaeElevator",
                        0.228);
        private static final LoggedTunableNumber l2_prep_pop_algae_wrist = new LoggedTunableNumber(
                        "Superstructure/L2PrepPopAlgaeWrist",
                        150.0);
                        //L3
        private static final LoggedTunableNumber l3_prep_pop_algae_elevator = new LoggedTunableNumber(
                        "Superstructure/L3PrepPopAlgaeElevator",
                        0.4);
        private static final LoggedTunableNumber l3_prep_pop_algae_wrist = new LoggedTunableNumber(
                        "Superstructure/L3PrepPopAlgaeWrist",
                        150.0);

        /** POP ALGAE **/
     
        private static final LoggedTunableNumber l2_pop_algae_elevator = new LoggedTunableNumber(
                        "Superstructure/L2PopAlgaeElevator",
                        0.3);
        private static final LoggedTunableNumber l2_pop_algae_wrist = new LoggedTunableNumber(
                        "Superstructure/L2PopAlgaeWrist",
                        147.0);
        
        private static final LoggedTunableNumber l3_pop_algae_elevator = new LoggedTunableNumber(
                        "Superstructure/L3PopAlgaeElevator",
                        0.5);
        private static final LoggedTunableNumber l3_pop_algae_wrist = new LoggedTunableNumber(
                        "Superstructure/L3PopAlgaeWrist",
                        120.0);

        /** SCORING **/

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

        private static final LoggedTunableNumber l4_coral_wrist_transition = new LoggedTunableNumber(
                        "Superstructure/L4CoralWristTransition",
                        40.0);

        private static final LoggedTunableNumber prep_climb_wrist = new LoggedTunableNumber(
                        "Superstructure/PrepClimbWrist",
                        200.0);
        private static final LoggedTunableNumber prep_climb_climb = new LoggedTunableNumber(
                        "Superstructure/PrepClimbClimb",
                        500.0);
        private static final LoggedTunableNumber deep_climb_climb = new LoggedTunableNumber(
                        "Superstructure/ExecuteDeepClimbClimb",
                        100.0);

        public enum SuperstructureState {
                HOME(() -> home_elevator.get(), () -> home_wrist.get(), () -> home_climb.get(), () -> home_intake.get()),
                STOW(() -> stow_elevator.get(), () -> stow_wrist.get(), () -> stow_climb.get(), () -> stow_intake.get()),

                INTAKE_CORAL(() -> intake_coral_elevator.get(), () -> intake_coral_wrist.get(), () -> stow_climb.get(), () -> intake_coral_intake.get()),
                HANDOFF_CORAL(() -> handoff_coral_elevator.get(), () -> handoff_coral_wrist.get(), () -> stow_climb.get(), () -> handoff_coral_intake.get()),

                L1_CORAL(() -> l1_coral_elevator.get(), () -> l1_coral_wrist.get(), () -> stow_climb.get(), () -> stow_intake.get()),
                L2_CORAL(() -> l2_coral_elevator.get(), () -> l2_coral_wrist.get(), () -> stow_climb.get(), () -> stow_intake.get()),
                L3_CORAL(() -> l3_coral_elevator.get(), () -> l3_coral_wrist.get(), () -> stow_climb.get(), () -> stow_intake.get()),
                L4_CORAL(() -> l4_coral_elevator.get(), () -> l4_coral_wrist.get(), () -> stow_climb.get(), () -> stow_intake.get()),
                L4_CORAL_TRANSITION(() -> l4_coral_elevator.get(), () -> l4_coral_wrist_transition.get(),() -> stow_climb.get(), () -> stow_intake.get()),

                L2_PREP_POP_ALGAE(() -> l2_prep_pop_algae_elevator.get(), () -> l2_prep_pop_algae_wrist.get(), () -> stow_climb.get(), () -> stow_intake.get()),
                L2_POP_ALGAE(() -> l2_pop_algae_elevator.get(), () -> l2_pop_algae_wrist.get(), () -> stow_climb.get(),  () -> stow_intake.get()),
                
                L3_PREP_POP_ALGAE(() -> l3_prep_pop_algae_elevator.get(), () -> l3_prep_pop_algae_wrist.get(), () -> stow_climb.get(), () -> stow_intake.get()),
                L3_POP_ALGAE(() -> l3_pop_algae_elevator.get(), () -> l3_pop_algae_wrist.get(), () -> stow_climb.get(), () -> stow_intake.get()),

                PREP_DEEP_CLIMB(() -> stow_elevator.get(), () -> prep_climb_wrist.get(), () -> prep_climb_climb.get(),  () -> stow_intake.get()),
                DEEP_CLIMB(() -> stow_elevator.get(), () -> prep_climb_wrist.get(), () -> deep_climb_climb.get(),  () -> stow_intake.get());

                private final DoubleSupplier elevatorSetpoint;
                private final DoubleSupplier wristSetpoint;
                private final DoubleSupplier climbSetpoint;
                private final DoubleSupplier intakeSetpoint;

                private SuperstructureState(DoubleSupplier elevatorSetpoint, DoubleSupplier wristSetpoint,
                                DoubleSupplier climbSetpoint, DoubleSupplier intakeSetpoint) {
                        this.elevatorSetpoint = elevatorSetpoint;
                        this.wristSetpoint = wristSetpoint;
                        this.climbSetpoint = climbSetpoint;
                        this.intakeSetpoint = intakeSetpoint;
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

                public double getIntakeDegrees() {
                        return intakeSetpoint.getAsDouble();
                }

                public DoubleSupplier getIntakeDegreesSupplier() {
                        return intakeSetpoint;
                }
        }
}
