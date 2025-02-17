// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
        @AutoLog
        class ElevatorIOInputs {
                public boolean motorConnected = true;
                public boolean followerConnected = true;
                public double positionMeters = 0.0;
                public double positionError = 0.0;
                public double velocityMetersPerSec = 0.0;
                public double[] appliedVolts = new double[] {};
                public double[] torqueCurrentAmps = new double[] {};
                public double[] supplyCurrentAmps = new double[] {};
                public double[] tempCelsius = new double[] {};
        }

        default void updateInputs(ElevatorIOInputs inputs) {
        }

        default void runOpenLoop(double output) {
        }

        default void runVolts(double volts) {
        }

        default void stop() {
        }

        /**
         * Run elevator output shaft to positionMeters
         */
        default void runPosition(double positionMeters) {
        }

        default void setPID(double kP, double kI, double kD) {
        }

        default void setSGVA(double kS, double kG, double kV, double kA) {
        }

        default void setKinematicConstraints(double maxVelocityMetersPerSec, double maxAccelerationMetersPerSec2) {
        }

        default void setBrakeMode(boolean enabled) {
        }
}