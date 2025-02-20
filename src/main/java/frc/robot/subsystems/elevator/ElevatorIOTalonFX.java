// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.elevator;

import static frc.lib.team6328.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;
import frc.robot.subsystems.Superstructure.SuperstructureState;

public class ElevatorIOTalonFX implements ElevatorIO {
        public static final double drumPitchDiameter = Units.inchesToMeters(2.4); // TODO: Fix this number
        public static final double reduction = 5.0 / (drumPitchDiameter * Math.PI);

        // Hardware
        private final TalonFX talon;
        private final TalonFX followerTalon;

        // Config
        private final TalonFXConfiguration config = new TalonFXConfiguration();

        // Status Signals
        private final StatusSignal<Angle> position;
        private final StatusSignal<AngularVelocity> velocity;
        private final StatusSignal<Double> positionError;
        private final StatusSignal<Voltage> appliedVolts;
        private final StatusSignal<Current> torqueCurrent;
        private final StatusSignal<Current> supplyCurrent;
        private final StatusSignal<Temperature> temp;
        private final StatusSignal<Voltage> followerAppliedVolts;
        private final StatusSignal<Current> followerTorqueCurrent;
        private final StatusSignal<Current> followerSupplyCurrent;
        private final StatusSignal<Temperature> followerTemp;

        // Open loop requests
        private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0.0);
        private final VoltageOut voltageRequest = new VoltageOut(0.0);
        // Closed loop requests
        private final MotionMagicVoltage motionMagicVoltageRequest = new MotionMagicVoltage(0.0);
        private final MotionMagicTorqueCurrentFOC motionMagicTorqueCurrentFOC = new MotionMagicTorqueCurrentFOC(0.0);

        private final Debouncer connectedDebouncer = new Debouncer(0.5);

        public ElevatorIOTalonFX() {

                talon = new TalonFX(31, "canivore"); 
                followerTalon = new TalonFX(32, "canivore"); 

                //talon = new TalonFX(33); // TODO: Fix this CAN ID
                //followerTalon = new TalonFX(34); // TODO: Fix this CAN ID

                followerTalon.setControl(new Follower(talon.getDeviceID(), true));

                // Configure motor
                config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
                config.Slot0 = new Slot0Configs().withKP(0).withKI(0).withKD(0);
                config.Feedback.SensorToMechanismRatio = reduction;
                config.Voltage.PeakForwardVoltage = 10.0;
                config.Voltage.PeakReverseVoltage = -10.0;
                config.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
                config.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
                config.CurrentLimits.StatorCurrentLimit = 80.0;
                config.CurrentLimits.StatorCurrentLimitEnable = true;
                config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.01;
                config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.01;
                config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.01;
                config.Audio.BeepOnBoot = false;
                config.Audio.AllowMusicDurDisable = true;
                config.Audio.BeepOnConfig = false;

                config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // TODO: Fix this based on thing
                tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));
                tryUntilOk(5, () -> talon.setPosition(SuperstructureState.HOME.getElevatorMeters(), 0.25));

                position = talon.getPosition();
                velocity = talon.getVelocity();
                positionError = talon.getClosedLoopError();
                appliedVolts = talon.getMotorVoltage();
                torqueCurrent = talon.getTorqueCurrent();
                supplyCurrent = talon.getSupplyCurrent();
                temp = talon.getDeviceTemp();

                followerAppliedVolts = followerTalon.getMotorVoltage();
                followerTorqueCurrent = followerTalon.getTorqueCurrent();
                followerSupplyCurrent = followerTalon.getSupplyCurrent();
                followerTemp = followerTalon.getDeviceTemp();

                BaseStatusSignal.setUpdateFrequencyForAll(
                                50.0, position, velocity,
                                positionError, appliedVolts, torqueCurrent, supplyCurrent, temp);
                ParentDevice.optimizeBusUtilizationForAll(talon);
        }

        @Override
        public void updateInputs(ElevatorIOInputs inputs) {
                boolean connected = BaseStatusSignal.refreshAll(
                                position, velocity,
                                positionError, appliedVolts, torqueCurrent, supplyCurrent, temp)
                                .isOK();
                boolean followerConnected = BaseStatusSignal.refreshAll(
                                followerAppliedVolts, followerTorqueCurrent, followerSupplyCurrent, followerTemp)
                                .isOK();

                inputs.motorConnected = connectedDebouncer.calculate(connected);
                inputs.followerConnected = connectedDebouncer.calculate(followerConnected);
                inputs.positionMeters = position.getValueAsDouble();
                inputs.velocityMetersPerSec = velocity.getValueAsDouble();
                inputs.positionError = positionError.getValueAsDouble();
                inputs.appliedVolts = new double[] { appliedVolts.getValueAsDouble(),
                                followerAppliedVolts.getValueAsDouble() };
                inputs.torqueCurrentAmps = new double[] { torqueCurrent.getValueAsDouble(),
                                followerTorqueCurrent.getValueAsDouble() };
                inputs.supplyCurrentAmps = new double[] { supplyCurrent.getValueAsDouble(),
                                followerSupplyCurrent.getValueAsDouble() };
                inputs.tempCelsius = new double[] { temp.getValueAsDouble(), followerTemp.getValueAsDouble() };
        }

        @Override
        public void stop() {
                talon.stopMotor();
        }

        @Override
        public void runOpenLoop(double output) {
                talon.setControl(torqueCurrentRequest.withOutput(output));
        }

        @Override
        public void runVolts(double volts) {
                talon.setControl(voltageRequest.withOutput(volts));
        }

        @Override
        public void runPosition(double positionMeters) {
                talon.setControl(motionMagicVoltageRequest.withPosition(positionMeters));
        }

        @Override
        public void setPosition(double positionMeters) {
                talon.setPosition(positionMeters);
        }

        @Override
        public void setPID(double kP, double kI, double kD) {
                config.Slot0.kP = kP;
                config.Slot0.kI = kI;
                config.Slot0.kD = kD;

                tryUntilOk(5, () -> talon.getConfigurator().apply(config));
        }

        @Override
        public void setSGVA(double kS, double kG, double kV, double kA) {
                config.Slot0.kS = kS;
                config.Slot0.kG = kG;
                config.Slot0.kV = kV;
                config.Slot0.kA = kA;

                tryUntilOk(5, () -> talon.getConfigurator().apply(config));
        }

        @Override
        public void setKinematicConstraints(double maxVelocityMetersPerSec, double maxAccelerationMetersPerSec2) {
                config.MotionMagic.MotionMagicCruiseVelocity = maxVelocityMetersPerSec;
                config.MotionMagic.MotionMagicAcceleration = maxAccelerationMetersPerSec2;

                tryUntilOk(5, () -> talon.getConfigurator().apply(config));
        }

        @Override
        public void setBrakeMode(boolean enabled) {
                new Thread(
                                () -> talon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast))
                                .start();
        }
}