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
import frc.robot.subsystems.elevator.ElevatorIOInputsAutoLogged;
import frc.robot.subsystems.flywheels.FlywheelsIO;
import frc.robot.subsystems.flywheels.FlywheelsIOInputsAutoLogged;
import frc.robot.util.Util;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheels extends SubsystemBase {
    // Tunable numbers
    private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP", 0.0);
    private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/kD", 0.0);

    private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS", 0.0);
    private static final LoggedTunableNumber kG = new LoggedTunableNumber("Flywheel/kG", 0.0);
    private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV", 0.0);
    private static final LoggedTunableNumber kA = new LoggedTunableNumber("Flywheel/kA", 0.0);

    private static final LoggedTunableNumber maxVelocityDegreesPerSec = new LoggedTunableNumber(
            "Flywheel/MaxVelocityDegreesPerSec", 360.0);
    private static final LoggedTunableNumber maxAccelerationDegreesPerSec2 = new LoggedTunableNumber(
            "Flywheel/MaxAccelerationDegreesPerSec2", 1080.0);
    
    //private static final LoggedTunableNumber intakingVolts = new LoggedTunableNumber("Flywheels/IntakingVolts", 8.0);
    //private static final LoggedTunableNumber ejectingVolts = new LoggedTunableNumber("Flywheels/EjectingVolts", -8.0);


    private final FlywheelsIO io;
    private final FlywheelsIOInputsAutoLogged inputs = new FlywheelsIOInputsAutoLogged();


    private final Alert motorDisconnectedAlert = new Alert("Flywheels motor disconnected!",
            Alert.AlertType.kWarning);

    @AutoLogOutput(key = "Flywheels/isBrakeMode")
    private boolean brakeModeEnabled = true;


    public Flywheels(FlywheelsIO io) {
        System.out.println("[Init] Instantiating Flywheels");

        super.setName("Flywheels");
        this.io = io;
        io.setPID(kP.get(), 0.0, kD.get());
        io.setSGVA(kS.get(), kG.get(), kV.get(), kA.get());
        io.setKinematicConstraints(maxVelocityDegreesPerSec.get(), maxAccelerationDegreesPerSec2.get());
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
        if (maxVelocityDegreesPerSec.hasChanged(hashCode())
                || maxAccelerationDegreesPerSec2.hasChanged(hashCode())) {
            io.setKinematicConstraints(maxVelocityDegreesPerSec.get(), maxAccelerationDegreesPerSec2.get());
        }
    }

/**
    public Command waitUntilAtSetpointCommand() {
        return new WaitUntilCommand(() -> atSetpoint());
    } */

    public void setBrakeMode(boolean enabled) {
        if (brakeModeEnabled == enabled)
            return;
        brakeModeEnabled = enabled;
        io.setBrakeMode(brakeModeEnabled);
    }

    

}