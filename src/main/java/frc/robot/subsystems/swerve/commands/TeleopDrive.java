package frc.robot.subsystems.swerve.commands;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.state.RobotStateEstimator;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.util.AllianceFlipUtil;

/**
 * Heading controller drive command.
 */
public class TeleopDrive extends Command {
    public TeleopDrive(DoubleSupplier throttle, DoubleSupplier strafe, DoubleSupplier turn, BooleanSupplier intakeLeft,
            BooleanSupplier intakeRight, BooleanSupplier snap) {
        mDrivetrain = RobotContainer.m_swerve;
        mThrottleSupplier = throttle;
        mStrafeSupplier = strafe;
        mTurnSupplier = turn;

        mLeftIntakeSupplier = intakeLeft;
        mRightIntakeSupplier = intakeRight;
        mSnapSupplier = snap;

        driveWithHeading.HeadingController.setPID(
                SwerveConstants.PID.kRotationkP,
                SwerveConstants.PID.kRotationkI,
                SwerveConstants.PID.kRotationkD);
        driveWithHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(mDrivetrain);
        setName("Teleop Drive");
    }

    private Swerve mDrivetrain;
    private DoubleSupplier mThrottleSupplier, mStrafeSupplier, mTurnSupplier;
    private BooleanSupplier mLeftIntakeSupplier, mRightIntakeSupplier, mSnapSupplier;
    private Optional<Rotation2d> mHeadingSetpoint = Optional.empty();
    private double mJoystickLastTouched = -1;

    private SwerveRequest.FieldCentric driveNoHeading = new SwerveRequest.FieldCentric()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);
    private SwerveRequest.FieldCentricFacingAngle driveWithHeading = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    @Override
    public void initialize() {
        mHeadingSetpoint = Optional.of(RobotStateEstimator.getInstance().getEstimatedPose().getRotation());
    }

    @Override
    public void execute() {
        double throttle = mThrottleSupplier.getAsDouble() * SwerveConstants.Kinematics.kTeleopLimits.maxLinearSpeed();
        double strafe = mStrafeSupplier.getAsDouble() * SwerveConstants.Kinematics.kTeleopLimits.maxLinearSpeed();
        double turnFieldFrame = mTurnSupplier.getAsDouble() * SwerveConstants.Kinematics.kTeleopLimits.maxOmega();
        double throttleFieldFrame = AllianceFlipUtil.shouldFlip() ? -throttle : throttle;
        double strafeFieldFrame = AllianceFlipUtil.shouldFlip() ? -strafe : strafe;

        if (Math.abs(turnFieldFrame) > 0.0) {
            mJoystickLastTouched = Timer.getFPGATimestamp();
        }

        boolean wantsLeftIntake = mLeftIntakeSupplier.getAsBoolean();
        boolean wantsRightIntake = mRightIntakeSupplier.getAsBoolean();
        boolean wantsSnap = mSnapSupplier.getAsBoolean();

        if (wantsLeftIntake || wantsRightIntake || wantsSnap) {
            double setpointDegrees = 0.0;
            if (wantsLeftIntake) {
                setpointDegrees = -54.0;
                mHeadingSetpoint = Optional.of(AllianceFlipUtil.apply(Rotation2d.fromDegrees(setpointDegrees)));
            } else if (wantsRightIntake) {
                setpointDegrees = 54.0;
                mHeadingSetpoint = Optional.of(AllianceFlipUtil.apply(Rotation2d.fromDegrees(setpointDegrees)));
            } else if (wantsSnap) {
                throttleFieldFrame *= 0.5; // slow down by 50% when snapping
                strafeFieldFrame *= 0.5;
                setpointDegrees = 45.0 * Math
                        .round(RobotStateEstimator.getInstance().getEstimatedPose().getRotation().getDegrees() / 45.0);
                mHeadingSetpoint = Optional.of(Rotation2d.fromDegrees(setpointDegrees));
            }

            mDrivetrain
                    .setControl(driveWithHeading.withVelocityX(throttleFieldFrame).withVelocityY(strafeFieldFrame)
                            .withTargetDirection(mHeadingSetpoint.get()));
            Logger.recordOutput("DriveMaintainHeading/Mode", "Heading");
            Logger.recordOutput("DriveMaintainHeading/HeadingSetpoint", mHeadingSetpoint.get().getDegrees());
        } else {
            // TODO: circumvent maintain heading to make driving a little smoother
            if (true || Math.abs(turnFieldFrame) > 0.0
                    || ((Timer.getFPGATimestamp() - mJoystickLastTouched < 0.25)
                            && Math.abs(mDrivetrain.getCurrentRobotChassisSpeeds().omegaRadiansPerSecond) > Math
                                    .toRadians(10))) {
                turnFieldFrame = turnFieldFrame * SwerveConstants.Kinematics.kTeleopLimits.maxOmega();
                mDrivetrain
                        .setControl(driveNoHeading.withVelocityX(throttleFieldFrame).withVelocityY(strafeFieldFrame)
                                .withRotationalRate(turnFieldFrame));
                mHeadingSetpoint = Optional.empty();
                Logger.recordOutput("DriveMaintainHeading/Mode", "NoHeading");
            } else {
                if (mHeadingSetpoint.isEmpty()) {
                    mHeadingSetpoint = Optional
                            .of(RobotStateEstimator.getInstance().getEstimatedPose().getRotation());
                }
                mDrivetrain
                        .setControl(
                                driveWithHeading.withVelocityX(throttleFieldFrame).withVelocityY(strafeFieldFrame)
                                        .withTargetDirection(mHeadingSetpoint.get()));
                Logger.recordOutput("DriveMaintainHeading/Mode", "Heading");
                Logger.recordOutput("DriveMaintainHeading/HeadingSetpoint", mHeadingSetpoint.get().getDegrees());
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}