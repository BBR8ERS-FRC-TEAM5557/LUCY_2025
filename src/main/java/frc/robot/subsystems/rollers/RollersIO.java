package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
    @AutoLog
    class RollersIOInputs {
        public boolean motorConnected = true;
        public boolean followerConnected = true;
        public double velocityRPM = 0.0;
        public double[] appliedVolts = new double[] {};
        public double[] torqueCurrentAmps = new double[] {};
        public double[] supplyCurrentAmps = new double[] {};
        public double[] tempCelsius = new double[] {};
    }

    default void updateInputs(RollersIOInputs inputs) {
    }

    default void runOpenLoop(double output) {
    }

    default void runVolts(double volts) {
    }

    default void stop() {
    }

    default void setBrakeMode(boolean enabled) {
    }

}