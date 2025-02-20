// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.util.Util;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
        // Tunable numbers
        private static final LoggedTunableNumber kP = new LoggedTunableNumber("Elevator/kP", 0.0);
        private static final LoggedTunableNumber kD = new LoggedTunableNumber("Elevator/kD", 0.0);

        private static final LoggedTunableNumber kS = new LoggedTunableNumber("Elevator/kS", 0.0);
        private static final LoggedTunableNumber kG = new LoggedTunableNumber("Elevator/kG", 0.0);
        private static final LoggedTunableNumber kV = new LoggedTunableNumber("Elevator/kV", 0.0);
        private static final LoggedTunableNumber kA = new LoggedTunableNumber("Elevator/kA", 0.0);

        private static final LoggedTunableNumber maxVelocityMetersPerSec = new LoggedTunableNumber(
                        "Elevator/MaxVelocityMetersPerSec", 2.0);
        private static final LoggedTunableNumber maxAccelerationMetersPerSec2 = new LoggedTunableNumber(
                        "Elevator/MaxAccelerationMetersPerSec2", 10);

        private static final LoggedTunableNumber homingVolts = new LoggedTunableNumber("Elevator/HomingVolts", -2.0);
        private static final LoggedTunableNumber homingTimeSecs = new LoggedTunableNumber("Elevator/HomingTimeSecs",
                        0.25);
        private static final LoggedTunableNumber homingVelocityThresh = new LoggedTunableNumber(
                        "Elevator/HomingVelocityThresh", 5.0);

        private static final LoggedTunableNumber setpointTolerance = new LoggedTunableNumber(
                        "Elevator/setpointTolerance", Units.inchesToMeters(0.5));

        private final ElevatorIO io;
        private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

        private final Alert motorDisconnectedAlert = new Alert("Elevator motor disconnected!",
                        Alert.AlertType.kWarning);

        @AutoLogOutput(key = "Elevator/isBrakeMode")         
        private boolean brakeModeEnabled = true;
        private boolean homed = false;
        private Debouncer homingDebouncer = new Debouncer(homingTimeSecs.get());

        private DoubleSupplier setpointMeters = () -> 0.0;

        public Elevator(ElevatorIO io) {
                System.out.println("[Init] Instantiating Elevator");

                super.setName("Elevator");
                this.io = io;
                io.setPID(kP.get(), 0.0, kD.get());
                io.setSGVA(kS.get(), kG.get(), kV.get(), kA.get());
                io.setKinematicConstraints(maxVelocityMetersPerSec.get(), maxAccelerationMetersPerSec2.get());
        }

        @Override
        public void periodic() {
                io.updateInputs(inputs);
                Logger.processInputs(this.getName(), inputs);

                motorDisconnectedAlert.set(!inputs.motorConnected);

                // Update tunable numbers
                if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
                        io.setPID(kP.get(), 0.0, kD.get());
                }
                if (kS.hasChanged(hashCode()) || kG.hasChanged(hashCode()) || kV.hasChanged(hashCode())
                                || kA.hasChanged(hashCode())) {
                        io.setSGVA(kS.get(), kG.get(), kV.get(), kA.get());
                }
                if (maxVelocityMetersPerSec.hasChanged(hashCode())
                                || maxAccelerationMetersPerSec2.hasChanged(hashCode())) {
                        io.setKinematicConstraints(maxVelocityMetersPerSec.get(), maxAccelerationMetersPerSec2.get());
                }
        }

        public Command runPositionCommand(DoubleSupplier setpointSupplier) {
                setpointMeters = setpointSupplier;
                return runEnd(() -> {
                        io.runPosition(setpointSupplier.getAsDouble());
                }, () -> {
                }).withName(getName() + " runPositionCommand");
        }

        public Command waitUntilAtSetpointCommand() {
                return new WaitUntilCommand(
                                () -> atSetpoint());
        }

        public void setBrakeMode(boolean enabled) {
                if (brakeModeEnabled == enabled)
                        return;
                brakeModeEnabled = enabled;
                io.setBrakeMode(brakeModeEnabled);
        }

        public double getPositionMeters() {
                return inputs.positionMeters;
        }

        @AutoLogOutput(key = "Elevator/SetpointMeters")
        public double getSetpointMeters() {
                return setpointMeters.getAsDouble();
        }

        @AutoLogOutput(key = "Elevator/atSetpoint")
        public boolean atSetpoint() {
                return Util.epsilonEquals(getPositionMeters(), getSetpointMeters(),
                                setpointTolerance.get());
        }

        @AutoLogOutput(key = "Elevator/homed")
        public boolean isHomed() {
                return homed;
        }

        public Command homingSequence() {
                return startRun(
                                () -> {
                                        homed = false;
                                        homingDebouncer = new Debouncer(homingTimeSecs.get());
                                        homingDebouncer.calculate(false);
                                },
                                () -> {
                                        if (!brakeModeEnabled) // implies limp mode
                                                return;
                                        io.runVolts(homingVolts.get());
                                        homed = homingDebouncer.calculate(
                                                        Math.abs(inputs.velocityMetersPerSec) <= homingVelocityThresh
                                                                        .get());
                                })
                                .until(() -> homed)
                                .andThen(
                                                () -> {
                                                        io.setPosition(SuperstructureState.HOME.getElevatorMeters());
                                                        io.runPosition(SuperstructureState.STOW.getElevatorMeters());
                                                });
        }
}