package frc.robot.subsystems.flywheels;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.wrist.WristIOInputsAutoLogged;

public interface FlywheelsIO {
    @AutoLog
    class FlywheelsIOInputs {
        public boolean motorConnected = true;
        public double velocityDegreesPerSec = 0.0;
        public double[] appliedVolts = new double[] {};
        public double[] torqueCurrentAmps = new double[] {};
        public double[] supplyCurrentAmps = new double[] {};
        public double[] tempCelsius = new double[] {};
    }

    default void updateInputs(WristIOInputsAutoLogged inputs) {
    }

    default void runOpenLoop(double output) {
    }

    default void runVolts(double volts) {
    }

    default void stop() {
    }

   

    default void setPID(double kP, double kI, double kD) {
    }

    default void setSGVA(double kS, double kG, double kV, double kA) {
    }

    default void setKinematicConstraints(double maxVelocityDegreesPerSec, double maxAccelerationDegreesPerSec2) {
    }

    default void setBrakeMode(boolean enabled) {
    }

    
}