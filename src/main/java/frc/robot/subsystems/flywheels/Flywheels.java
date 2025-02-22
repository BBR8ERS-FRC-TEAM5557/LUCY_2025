// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheels;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.LoggedTunableNumber;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends SubsystemBase {
    // Tunable numbers
    private static final LoggedTunableNumber mIdleRpm = new LoggedTunableNumber("Flywheels/IdleVolts", 1.0);

    private static final LoggedTunableNumber mIntakingCoralRpm = new LoggedTunableNumber("Flywheels/IntakingCoralVolts",
            12.0);
    private static final LoggedTunableNumber mHoldingCoralRpm = new LoggedTunableNumber("Flywheels/HoldingCoralVolts",
            2.0);
    private static final LoggedTunableNumber mScoringCoralRpm = new LoggedTunableNumber("Flywheels/EjectingCoralVolts",
            -5.0);

    private static final LoggedTunableNumber mStallVelocityThreshold = new LoggedTunableNumber(
            "Flywheels/StallVelocityThresholdRPM",
            50);
    private static final LoggedTunableNumber mStallCurrentThreshold = new LoggedTunableNumber(
            "Flywheels/StallCurrentThresholdAmps",
            30.0);
    private static final LoggedTunableNumber mStallTime = new LoggedTunableNumber(
            "Flywheels/StallTimeSecs",
            0.25);

    private final FlywheelsIO io;
    private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();

    private State mState = State.IDLE;
    private Debouncer stallDebouncer = new Debouncer(mStallTime.get());

    @AutoLogOutput(key = "Flywheels/stalled")
    private boolean stalled = false;

    @AutoLogOutput(key = "Flywheels/isBrakeMode")
    private boolean brakeModeEnabled = true;

    private final Alert motorDisconnectedAlert = new Alert("Flywheels motor disconnected!",
            Alert.AlertType.kWarning);

    public enum State {
        STOP(() -> 0.0),
        IDLE(mIdleRpm),

        INTAKE_CORAL(mIntakingCoralRpm),
        HOLD_CORAL(mHoldingCoralRpm),
        SCORE_CORAL(mScoringCoralRpm);

        private State(DoubleSupplier voltageSupplier) {
            volts = voltageSupplier;
        }

        private final DoubleSupplier volts;

        private double getVolts() {
            return volts.getAsDouble();
        }
    }

    public Flywheels(FlywheelsIO io) {
        System.out.println("[Init] Instantiating Flywheels");

        super.setName("Flywheels");
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(this.getName(), inputs);

        motorDisconnectedAlert.set(!inputs.motorConnected);

        io.runVolts(mState.getVolts());

        Logger.recordOutput("Flywheels/State", mState);
        Logger.recordOutput("Flywheels/SetpointVolts", mState.getVolts());
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
                .withName("FlywheelsIntakeCoral");
    }

    public Command scoreCoral() {
        return startEnd(() -> setState(State.SCORE_CORAL), () -> setState(State.IDLE))
                .withName("FlywheelsScoreCoral");
    }

    public Command stop() {
        return runOnce(() -> setState(State.STOP));
    }

}