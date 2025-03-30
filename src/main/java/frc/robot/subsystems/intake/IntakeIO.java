package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
    @AutoLog
    class IntakeIOInputs {
        public boolean motorConnected = true;
        public double positionDegrees = 0.0;
        public double positionError = 0.0;
        public double velocityDegreesPerSec = 0.0;
        public double[] appliedVolts = new double[] {};
        public double[] torqueCurrentAmps = new double[] {};
        public double[] supplyCurrentAmps = new double[] {};
        public double[] tempCelsius = new double[] {};
    }

    default void updateInputs(IntakeIOInputs inputs) {
    }

    default void runOpenLoop(double output) {
    }

    default void runVolts(double volts) {
    }

    default void stop() {
    }

    /**
     * Run Intake output shaft to positionDegrees
     */
    default void runPosition(double positionDegrees, double feedforward) {
    }

    default void setPosition(double positionDegrees) {
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