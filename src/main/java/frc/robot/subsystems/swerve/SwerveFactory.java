package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotContainer;

public class SwerveFactory {
    Swerve swerve = RobotContainer.m_swerve;

    // public Command sysIdDriveQuasiCommand(Direction direction) {
    // return translationSysIdRoutine.quasistatic(direction).withName("SysId Drive
    // Quasistatic " + direction)
    // .finallyDo(() -> swerve.setControl(new SwerveRequest.ApplyChassisSpeeds()));
    // }

    // public Command sysIdDriveDynamCommand(SysIdRoutine.Direction direction) {
    // return translationSysIdRoutine.dynamic(direction).withName("SysId Drive
    // Dynamic " + direction)
    // .finallyDo(() -> swerve.setControl(new SwerveRequest.ApplyChassisSpeeds()));
    // }

    // public Command sysIdSteerQuasiCommand(Direction direction) {
    // return steerSysIdRoutine.quasistatic(direction).withName("SysId Steer
    // Quasistatic " + direction)
    // .finallyDo(() -> swerve.setControl(new SwerveRequest.ApplyChassisSpeeds()));
    // }

    // public Command sysIdSteerDynamCommand(SysIdRoutine.Direction direction) {
    // return steerSysIdRoutine.dynamic(direction).withName("SysId Steer Dynamic " +
    // direction)
    // .finallyDo(() -> swerve.setControl(new SwerveRequest.ApplyChassisSpeeds()));
    // }

    // public Command sysIdRotationDynamCommand(SysIdRoutine.Direction direction) {
    // return rotationSysIdRoutine.dynamic(direction).withName("SysId Rotate Dynamic
    // " + direction)
    // .finallyDo(() -> swerve.setControl(new SwerveRequest.ApplyChassisSpeeds()));
    // }

    // public Command sysIdRotationQuasiCommand(SysIdRoutine.Direction direction) {
    // return rotationSysIdRoutine.quasistatic(direction).withName("SysId Rotate
    // Quasistatic " + direction)
    // .finallyDo(() -> swerve.setControl(new SwerveRequest.ApplyChassisSpeeds()));
    // }

    // public Command sysIdDriveSlipCommand() {
    // return
    // slipSysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward).withName("SysId
    // Drive Slip")
    // .finallyDo(() -> swerve.setControl(new SwerveRequest.ApplyChassisSpeeds()));
    // }

    // private final SwerveRequest.SysIdSwerveTranslation
    // translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    // private final SwerveRequest.SysIdSwerveRotation rotationCharacterization =
    // new SwerveRequest.SysIdSwerveRotation();
    // private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization = new
    // SwerveRequest.SysIdSwerveSteerGains();

    // private final SysIdRoutine translationSysIdRoutine = new SysIdRoutine(
    // new SysIdRoutine.Config(null, null, null,
    // SysIdRoutineSignalLogger.logState()),
    // new SysIdRoutine.Mechanism((volts) ->
    // swerve.setControl(translationCharacterization.withVolts(volts)), null,
    // swerve));

    // private final SysIdRoutine steerSysIdRoutine = new SysIdRoutine(
    // new SysIdRoutine.Config(null, Volts.of(3), null,
    // SysIdRoutineSignalLogger.logState()),
    // new SysIdRoutine.Mechanism((volts) ->
    // swerve.setControl(steerCharacterization.withVolts(volts)), null,
    // swerve));

    // private final SysIdRoutine rotationSysIdRoutine = new SysIdRoutine(
    // new SysIdRoutine.Config(Volts.of(0.25).per(Second), null, null,
    // SysIdRoutineSignalLogger.logState()),
    // new SysIdRoutine.Mechanism((volts) ->
    // swerve.setControl(rotationCharacterization.withVolts(volts)), null,
    // swerve));

    // private final SysIdRoutine slipSysIdRoutine = new SysIdRoutine(
    // new SysIdRoutine.Config(Volts.of(0.25).per(Second), null, null,
    // SysIdRoutineSignalLogger.logState()),
    // new SysIdRoutine.Mechanism((volts) ->
    // swerve.setControl(translationCharacterization.withVolts(volts)), null,
    // swerve));
}
