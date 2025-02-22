package frc.robot.subsystems.flywheels;

import org.littletonrobotics.junction.AutoLog;

public interface FlywheelsIO {
    @AutoLog
    class FlywheelsIOInputs {
        public boolean motorConnected = true;
        public double velocityRPM = 0.0;
        public double[] appliedVolts = new double[] {};
        public double[] torqueCurrentAmps = new double[] {};
        public double[] supplyCurrentAmps = new double[] {};
        public double[] tempCelsius = new double[] {};
    }

    default void updateInputs(FlywheelsIOInputs inputs) {
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