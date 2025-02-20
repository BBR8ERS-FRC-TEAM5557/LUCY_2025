package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import frc.lib.team6328.LoggedTunableNumber;

public class Superstructure {
    private static final LoggedTunableNumber home_elevator = new LoggedTunableNumber(
            "Superstructure/HomeElevator",
            0.0);
    private static final LoggedTunableNumber home_wrist = new LoggedTunableNumber(
            "Superstructure/HomeWrist",
            -45.0);
    private static final LoggedTunableNumber stow_elevator = new LoggedTunableNumber(
            "Superstructure/StartElevator",
            0.0);
    private static final LoggedTunableNumber stow_wrist = new LoggedTunableNumber(
            "Superstructure/StartWrist",
            -45.0);

    private static final LoggedTunableNumber intake_coral_elevator = new LoggedTunableNumber(
            "Superstructure/IntakeCoralElevator",
            0.2);
    private static final LoggedTunableNumber intake_coral_wrist = new LoggedTunableNumber(
            "Superstructure/IntakeCoralWrist",
            -45.0);

    private static final LoggedTunableNumber l1_coral_elevator = new LoggedTunableNumber(
            "Superstructure/L1CoralElevator",
            0.0);
    private static final LoggedTunableNumber l1_coral_wrist = new LoggedTunableNumber(
            "Superstructure/L1CoralWrist",
            128.0);
    private static final LoggedTunableNumber l2_coral_elevator = new LoggedTunableNumber(
            "Superstructure/L2CoralElevator",
            0.0);
    private static final LoggedTunableNumber l2_coral_wrist = new LoggedTunableNumber(
            "Superstructure/L2CoralWrist",
            -45.0);

    private static final LoggedTunableNumber l3_coral_elevator = new LoggedTunableNumber(
                "Superstructure/L3CoralElevator",
                0.0);
    private static final LoggedTunableNumber l3_coral_wrist = new LoggedTunableNumber(
                "Superstructure/L3CoralWrist",
                175.0);
    private static final LoggedTunableNumber l4_coral_elevator = new LoggedTunableNumber(
                        "Superstructure/L4CoralElevator",
                        3.0);
    private static final LoggedTunableNumber l4_coral_wrist = new LoggedTunableNumber(
                        "Superstructure/L4CoralWrist",
                        -45.0);

    private static final LoggedTunableNumber intake_coral_flywheels = new LoggedTunableNumber(
        "Superstructure/IntakeFlywheels", 8.0);
    private static final LoggedTunableNumber eject_coral_flywheels = new LoggedTunableNumber(
        "Superstructure/EjectFlywheels", -8.0);
    private static final LoggedTunableNumber idle_coral_flywheels = new LoggedTunableNumber(
        "Superstructure/IdleFlywheels", 2.0);

    public enum SuperstructureState {
        HOME(() -> home_elevator.get(), () -> home_wrist.get(), () -> idle_coral_flywheels.get()),
        STOW(() -> stow_elevator.get(), () -> stow_wrist.get(), () -> idle_coral_flywheels.get()),

        INTAKE_CORAL(() -> intake_coral_elevator.get(), () -> intake_coral_wrist.get(), () -> intake_coral_flywheels.get()),

        L1_CORAL(() -> l1_coral_elevator.get(), () -> l1_coral_wrist.get(), () -> idle_coral_flywheels.get()), 
        L2_CORAL(() -> l2_coral_elevator.get(), () -> l2_coral_wrist.get(), () -> idle_coral_flywheels.get()),
        L3_CORAL(() -> l3_coral_elevator.get(), () -> l3_coral_wrist.get(), () -> idle_coral_flywheels.get()),
        L4_CORAL(() -> l4_coral_elevator.get(), () -> l4_coral_wrist.get(), () -> idle_coral_flywheels.get());


        // TODO: Add the remaining states plz

        private final DoubleSupplier elevatorSetpoint;
        private final DoubleSupplier wristSetpoint;
        private DoubleSupplier flywheelSetpoint;
        
                private SuperstructureState(DoubleSupplier elevatorSetpoint, DoubleSupplier wristSetpoint, DoubleSupplier flywheelSetpoint) {
                    this.elevatorSetpoint = elevatorSetpoint;
                    this.wristSetpoint = wristSetpoint;
                    this.flywheelSetpoint = flywheelSetpoint;
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

        public double getFlywheelVolts() {
            return flywheelSetpoint.getAsDouble();
        }

        public DoubleSupplier getFlywheelVoltsSupplier() {
            return flywheelSetpoint;
        }
    }
}
