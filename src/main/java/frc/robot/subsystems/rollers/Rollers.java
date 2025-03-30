// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.rollers;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Rollers extends SubsystemBase {
    // Tunable numbers
    private static final LoggedTunableNumber mIdleRpm = new LoggedTunableNumber("Roller/IdleVolts", 0.0);

    private static final LoggedTunableNumber mIntakingCoralRpm = new LoggedTunableNumber("Rollers/IntakingCoralVolts",
            5.0);
    private static final LoggedTunableNumber mHoldingCoralRpm = new LoggedTunableNumber("Rollers/HoldingCoralVolts",
            1.0);
    private static final LoggedTunableNumber mHandoffCoralRpm = new LoggedTunableNumber("Rollers/EjectingCoralVolts",
            -5.0);

    private static final LoggedTunableNumber mStallVelocityThreshold = new LoggedTunableNumber(
            "Rollers/StallVelocityThresholdRPM",
            50);
    private static final LoggedTunableNumber mStallCurrentThreshold = new LoggedTunableNumber(
            "Rollers/StallCurrentThresholdAmps",
            30.0);
    private static final LoggedTunableNumber mStallTime = new LoggedTunableNumber(
            "Rollers/StallTimeSecs",
            0.25);

    private final RollersIO io;
    private final RollersIOInputsAutoLogged inputs = new RollersIOInputsAutoLogged();

    private State mState = State.IDLE;
    private Debouncer stallDebouncer = new Debouncer(mStallTime.get());

    @AutoLogOutput(key = "Rollers/stalled")
    private boolean stalled = false;

    @AutoLogOutput(key = "Rollers/isBrakeMode")
    private boolean brakeModeEnabled = true;

    private final Alert motorDisconnectedAlert = new Alert("Rollers motor disconnected!",
            Alert.AlertType.kWarning);

    public enum State {
        STOP(() -> 0.0),
        IDLE(mIdleRpm),

        INTAKE_CORAL(mIntakingCoralRpm),
        HOLD_CORAL(mHoldingCoralRpm),
        HANDOFF_CORAL(mHandoffCoralRpm);

        private State(DoubleSupplier voltageSupplier) {
            volts = voltageSupplier;
        }

        /**
        private State(DoubleSupplier intakeVoltageSupplier, DoubleSupplier flywheelDoubleSupplier) {

        } */

        private final DoubleSupplier volts;

        private double getVolts() {
            return volts.getAsDouble();
        }
    }

    public Rollers(RollersIO io) {
        System.out.println("[Init] Instantiating Rollers");

        super.setName("Rollers");
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(this.getName(), inputs);

        motorDisconnectedAlert.set(!inputs.motorConnected);

        io.runVolts(mState.getVolts());

        Logger.recordOutput("Rollers/State", mState);
        Logger.recordOutput("Rollers/SetpointVolts", mState.getVolts());
    }

    private void setState(State state) {
        this.mState = state;
    }

    public void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled)
            return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }

    public Command intakeCoral() {
        return startRun(
                () -> {
                    setState(State.INTAKE_CORAL);
                    stallDebouncer = new Debouncer(mStallTime.get());
                    stallDebouncer.calculate(false);
                    stalled = false;
                },
                () -> {
                    stalled = stallDebouncer.calculate(inputs.velocityRPM < mStallVelocityThreshold.get()
                            && inputs.torqueCurrentAmps[0] > mStallCurrentThreshold.get());
                })
                .until(() -> stalled)
                .finallyDo(() -> {
                    if (stalled) {
                        setState(State.HOLD_CORAL);
                    } else {
                        setState(State.IDLE);
                    }
                })
                .withName("RollersIntakeCoral");
    }

    public Command intakeCoralManual() {
        return startEnd(() -> setState(State.INTAKE_CORAL), () -> setState(State.IDLE))
                .withName("RollersIntakeCoralManually");
    }

    public Command scoreCoral() {
        return startEnd(() -> setState(State.HANDOFF_CORAL), () -> setState(State.IDLE))
                .withName("RollersScoreCoral");
    }

    public Command handoffCoral() {
        return startEnd(() -> setState(State.HANDOFF_CORAL), () -> setState(State.IDLE))
                .withName("RollersHandoffCoral");
    }

    public Command stop() {
        return runOnce(() -> setState(State.STOP));
    }

}