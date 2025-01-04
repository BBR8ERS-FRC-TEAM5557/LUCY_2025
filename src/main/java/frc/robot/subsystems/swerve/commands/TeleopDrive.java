package frc.robot.subsystems.swerve.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.state.RobotStateEstimator;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Util;

/**
 * Heading controller drive command.
 */
public class TeleopDrive extends Command {
    public TeleopDrive(DoubleSupplier throttle, DoubleSupplier strafe, DoubleSupplier turn) {
        mDrivetrain = RobotContainer.m_swerve;
        mThrottleSupplier = throttle;
        mStrafeSupplier = strafe;
        mTurnSupplier = turn;

        driveWithHeading.HeadingController.setPID(
                SwerveConstants.PID.kRotationkP,
                SwerveConstants.PID.kRotationkI,
                SwerveConstants.PID.kRotationkD);
        driveWithHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        // uncomment for easy PID tuning
        // SmartDashboard.putData("Heading Controller PID",
        // driveWithHeading.HeadingController);

        addRequirements(mDrivetrain);
        setName("Teleop Drive Maintain Heading");

        if (Robot.isSimulation()) {
            driveNoHeading.DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
            driveWithHeading.DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
        }
    }

    private Swerve mDrivetrain;
    private DoubleSupplier mThrottleSupplier, mStrafeSupplier, mTurnSupplier;
    private Optional<Rotation2d> mHeadingSetpoint = Optional.empty();
    private double mJoystickLastTouched = -1;

    private SwerveRequest.FieldCentric driveNoHeading = new SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    private SwerveRequest.FieldCentricFacingAngle driveWithHeading = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    @Override
    public void initialize() {
        mHeadingSetpoint = Optional.empty();
    }

    @Override
    public void execute() {
        double throttle = mThrottleSupplier.getAsDouble() * SwerveConstants.Kinematics.kTeleopLimits.maxLinearSpeed();
        double strafe = mStrafeSupplier.getAsDouble() * SwerveConstants.Kinematics.kTeleopLimits.maxLinearSpeed();
        double turnFieldFrame = mTurnSupplier.getAsDouble();
        double throttleFieldFrame = AllianceFlipUtil.shouldFlip() ? -throttle : throttle;
        double strafeFieldFrame = AllianceFlipUtil.shouldFlip() ? -strafe : strafe;

        if (Math.abs(turnFieldFrame) > 0.0) {
            mJoystickLastTouched = Timer.getFPGATimestamp();
        }

        if (Math.abs(turnFieldFrame) > 0.0
                || (Util.epsilonEquals(mJoystickLastTouched, Timer.getFPGATimestamp(), 0.25)
                        && Math.abs(mDrivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond) > Math
                                .toRadians(10))) {
            turnFieldFrame = turnFieldFrame * SwerveConstants.Kinematics.kTeleopLimits.maxAlpha();
            mDrivetrain.setControl(driveNoHeading.withVelocityX(throttleFieldFrame).withVelocityY(strafeFieldFrame)
                    .withRotationalRate(turnFieldFrame));
            mHeadingSetpoint = Optional.empty();
            Logger.recordOutput("DriveMaintainHeading/Mode", "NoHeading");
        } else {
            if (mHeadingSetpoint.isEmpty()) {
                mHeadingSetpoint = Optional.of(RobotStateEstimator.getInstance().getEstimatedPose().getRotation());
            }
            mDrivetrain.setControl(driveWithHeading.withVelocityX(throttleFieldFrame).withVelocityY(strafeFieldFrame)
                    .withTargetDirection(mHeadingSetpoint.get()));
            Logger.recordOutput("DriveMaintainHeading/Mode", "Heading");
            Logger.recordOutput("DriveMaintainHeading/HeadingSetpoint", mHeadingSetpoint.get().getDegrees());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}