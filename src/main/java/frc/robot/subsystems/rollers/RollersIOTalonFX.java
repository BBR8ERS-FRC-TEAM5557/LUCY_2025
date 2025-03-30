package frc.robot.subsystems.rollers;

import static frc.lib.team6328.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.*;

public class RollersIOTalonFX implements RollersIO {
    // Hardware
    private final TalonFX talon;
    private final TalonFX followerTalon;

    // Config
    private final TalonFXConfiguration config = new TalonFXConfiguration();

    // Status Signals
    private final StatusSignal<Angle> position;
    private final StatusSignal<AngularVelocity> velocity;
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

    private final Debouncer connectedDebouncer = new Debouncer(0.5);

    public RollersIOTalonFX() {
        talon = new TalonFX(40);
        followerTalon = new TalonFX(41);

        followerTalon.setControl(new Follower(talon.getDeviceID(), false));

        // Configure motor
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Voltage.PeakForwardVoltage = 12.0; //
        config.Voltage.PeakReverseVoltage = -12.0; //
        config.CurrentLimits.StatorCurrentLimit = 40.0; //
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = 0.01;
        config.ClosedLoopRamps.TorqueClosedLoopRampPeriod = 0.04;
        config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.01;
        config.Audio.BeepOnBoot = false;
        config.Audio.BeepOnConfig = true;
        

        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        tryUntilOk(5, () -> talon.getConfigurator().apply(config, 0.25));

        position = talon.getPosition();
        velocity = talon.getVelocity();
        appliedVolts = talon.getMotorVoltage();
        torqueCurrent = talon.getTorqueCurrent();
        supplyCurrent = talon.getSupplyCurrent();
        temp = talon.getDeviceTemp();

        followerAppliedVolts = followerTalon.getMotorVoltage();
        followerTorqueCurrent = followerTalon.getTorqueCurrent();
        followerSupplyCurrent = followerTalon.getSupplyCurrent();
        followerTemp = followerTalon.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0, velocity, position,
                appliedVolts, torqueCurrent, supplyCurrent, temp);
        ParentDevice.optimizeBusUtilizationForAll(talon);
    }

    // @Override
    public void updateInputs(RollersIOInputs inputs) {
        boolean connected = BaseStatusSignal.refreshAll(
                velocity, position,
                appliedVolts, torqueCurrent, supplyCurrent, temp)
                .isOK();
        boolean followerConnected = BaseStatusSignal.refreshAll(
                followerAppliedVolts, followerTorqueCurrent, followerSupplyCurrent, followerTemp)
                .isOK();

        inputs.motorConnected = connectedDebouncer.calculate(connected);
        inputs.followerConnected = connectedDebouncer.calculate(followerConnected);
        inputs.velocityRPM = velocity.getValueAsDouble();
        inputs.appliedVolts = new double[] { appliedVolts.getValueAsDouble(),
            followerAppliedVolts.getValueAsDouble() };
        inputs.torqueCurrentAmps = new double[] { torqueCurrent.getValueAsDouble(),
            followerTorqueCurrent.getValueAsDouble() };
        inputs.supplyCurrentAmps = new double[] { supplyCurrent.getValueAsDouble(),
            followerSupplyCurrent.getValueAsDouble() };
        inputs.tempCelsius = new double[] { temp.getValueAsDouble(), 
            followerTemp.getValueAsDouble() };
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
    public void setBrakeMode(boolean enabled) {
        new Thread(
                () -> talon.setNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast))
                .start();
    }
}