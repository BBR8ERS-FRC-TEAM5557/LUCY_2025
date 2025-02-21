// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.flywheels;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.subsystems.Superstructure.SuperstructureState;
import frc.robot.subsystems.wrist.WristIOInputsAutoLogged;
import frc.robot.util.Util;
import lombok.RequiredArgsConstructor;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends SubsystemBase {
    // Tunable numbers
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP", 0.0);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/kD", 0.0);

    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS", 0.0);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV", 0.0);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheel/kA", 0.0);

    private static final LoggedTunableNumber mIdleRpm = new LoggedTunableNumber("Flywheels/IdleRpm", 2500.0);
    private static final LoggedTunableNumber mIntakingRpm = new LoggedTunableNumber("Flywheels/IntakingRpm", -2000.0);
    private static final LoggedTunableNumber mEjectingRpm = new LoggedTunableNumber("Flywheels/EjectingRpm", 2500.0);
    private static final LoggedTunableNumber mScoringRpm = new LoggedTunableNumber("Flywheels/EjectingRpm", 2500.0);


    private final FlywheelsIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();


    private State mState = State.IDLE;
    private IdleMode mIdleMode = IdleMode.TELEOP;

    private final Alert motorDisconnectedAlert = new Alert("Flywheels motor disconnected!",
            Alert.AlertType.kWarning);

    @RequiredArgsConstructor
    public enum State {
        STOP(() -> 0.0),
        IDLE(mIdleRpm),
        INTAKE(mIntakingRpm),
        EJECT(mEjectingRpm),
        SCORE(mScoringRpm),

        CHARACTERIZING(() -> 0.0);

        private final DoubleSupplier rpm;

        private double getRPM() {
            return rpm.getAsDouble();
        }
    }

    public enum IdleMode {
        TELEOP,
        AUTO
    }

    @AutoLogOutput(key = "Flywheels/isBrakeMode")
    private boolean brakeModeEnabled = true;


    public Flywheels(FlywheelsIO io) {
        System.out.println("[Init] Instantiating Flywheels");

        super.setName("Flywheels");
        this.io = io;
        io.setPID(kP.get(), 0.0, kD.get());
        io.setSGVA(kS.get(), kV.get(), kA.get());

        setDefaultCommand(runOnce(() -> setState(State.IDLE)).withName("Flywheels Idle"));
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
        if (kS.hasChanged(hashCode()) || kV.hasChanged(hashCode())
                || kA.hasChanged(hashCode())) {
            io.setSGVA(kS.get(), kV.get(), kA.get());
        }
       
        Logger.recordOutput("Flywheels/State", mState);
        Logger.recordOutput("Flywheels/SetpointRpm", mState.getRPM());
    }

   
    private void setState(State state) {
        this.mState = state;
    }
/**
    public Command waitUntilAtSetpointCommand() {
        return new WaitUntilCommand(() -> atSetpoint());
    } */

    public void setIdleMode(IdleMode idleMode) {
        if (this.mIdleMode != idleMode) {
            // Idle after switching IdleMode
            this.mIdleMode = idleMode;
            idle();
        }
    }

    //TODO: Change auto scoring idle mode
    private void idle() {
        // Change based on current idle mode
        if (mIdleMode == IdleMode.TELEOP) {
            setState(State.IDLE);
        } else if (mIdleMode == IdleMode.AUTO) {
            setState(State.SCORE);
        }

    }

    public void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled)
            return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }



public Command intakeCoralSubstation() {
    return startEnd(() -> setState(State.INTAKE), () -> setState(State.IDLE))
                .withName("FlywheelsIntake");
}

public Command scoreCoral() {
    return startEnd(() -> setState(State.SCORE), () -> setState(State.IDLE))
        .withName("FlywheelsScoreCoral");
}

public Command ejectCoral() {
    return startEnd(() -> setState(State.EJECT), () -> setState(State.IDLE))
        .withName("FlywheelsEject");
}

public Command stop() {
    return runOnce(() -> setState(State.STOP));
}


}