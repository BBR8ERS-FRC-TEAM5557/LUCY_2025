// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.filter.Debouncer;
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

public class Intake extends SubsystemBase {
    // Tunable numbers
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Intake/kP", 0.2);
    private static final LoggedTunableNumber kI = new LoggedTunableNumber("Intake/kI", 0.002);
    
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Intake/kD", 0.00002);

    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Intake/kS", 0.01);
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Intake/kG", -.45);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Intake/kV", 0.0);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Intake/kA", 0.0);

    private static final LoggedTunableNumber maxVelocityDegreesPerSec = new LoggedTunableNumber(
            "Intake/MaxVelocityDegreesPerSec", 720.0);
    private static final LoggedTunableNumber maxAccelerationDegreesPerSec2 = new LoggedTunableNumber(
            "Intake/MaxAccelerationDegreesPerSec2", 2000.0);

    private static final LoggedTunableNumber homingVolts = new LoggedTunableNumber("Intake/HomingVolts", -2.0);
    private static final LoggedTunableNumber homingTimeSecs = new LoggedTunableNumber("Intake/HomingTimeSecs",
            0.25);
    private static final LoggedTunableNumber homingVelocityThresh = new LoggedTunableNumber(
            "Intake/HomingVelocityThresh", 0.1);

    private static final LoggedTunableNumber setpointTolerance = new LoggedTunableNumber(
            "Intake/setpointTolerance", 1.5);

    private final IntakeIO io;
    private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

    private final Alert motorDisconnectedAlert = new Alert("Intake motor disconnected!",
            Alert.AlertType.kWarning);

    @AutoLogOutput(key = "Intake/isBrakeMode")
    private boolean brakeModeEnabled = true;
    private boolean homed = false;
    private Debouncer homingDebouncer = new Debouncer(homingTimeSecs.get());

    private DoubleSupplier setpointDegrees = () -> 0.0;

    public Intake(IntakeIO io) {
        System.out.println("[Init] Instantiating Intake");

        super.setName("Intake");
        this.io = io;
        io.setPID(kP.get(), kI.get(), kD.get());
        io.setSGVA(kS.get(), 0.0, kV.get(), kA.get());
        io.setKinematicConstraints(maxVelocityDegreesPerSec.get(), maxAccelerationDegreesPerSec2.get());
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs(this.getName(), inputs);

        motorDisconnectedAlert.set(!inputs.motorConnected);

        // Update tunable numbers
        if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode()) || kI.hasChanged(hashCode())) {
            io.setPID(kP.get(), kI.get(), kD.get());
        }
        if (kS.hasChanged(hashCode()) || kG.hasChanged(hashCode()) || kV.hasChanged(hashCode())
                || kA.hasChanged(hashCode())) {
            io.setSGVA(kS.get(), kG.get(), kV.get(), kA.get());
        }
        if (maxVelocityDegreesPerSec.hasChanged(hashCode())
                || maxAccelerationDegreesPerSec2.hasChanged(hashCode())) {
            io.setKinematicConstraints(maxVelocityDegreesPerSec.get(), maxAccelerationDegreesPerSec2.get());
        }
    }

    public Command runPositionCommand(DoubleSupplier setpointSupplierDegrees) {
        setpointDegrees = setpointSupplierDegrees;
        return runEnd(() -> {
            io.runPosition(setpointSupplierDegrees.getAsDouble(),
                    kG.get() * Math.cos(Math.toRadians(getPositionDegrees())));
        }, () -> {
        }).withName(getName() + " runPositionCommand");
    }

    public Command waitUntilAtSetpointCommand() {
        return new WaitUntilCommand(() -> atSetpoint());
    }

    public void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled)
            return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }

    public double getPositionDegrees() {
        return inputs.positionDegrees;
    }

    @AutoLogOutput(key = "Intake/SetpointDegrees")
    public double getSetpointDegrees() {
        return setpointDegrees.getAsDouble();
    }

    @AutoLogOutput(key = "Intake/atSetpoint")
    public boolean atSetpoint() {
        return Util.epsilonEquals(getPositionDegrees(), getSetpointDegrees(),
                setpointTolerance.get());
    }

    @AutoLogOutput(key = "Intake/homed")
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
                    if (!brakeModeEnabled) // implies in limp mode
                        return;
                    io.runVolts(homingVolts.get());
                    homed = homingDebouncer.calculate(
                            Math.abs(inputs.velocityDegreesPerSec) <= homingVelocityThresh
                                    .get());
                })
                .until(() -> homed)
                .andThen(
                        () -> {
                            io.setPosition(SuperstructureState.HOME.getIntakeDegrees());
                            runPositionCommand(() -> SuperstructureState.STOW.getIntakeDegrees()).schedule();
                        });
    }
}