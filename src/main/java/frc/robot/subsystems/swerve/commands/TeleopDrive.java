package frc.robot.subsystems.swerve.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.team6328.AllianceFlipUtil;
import frc.robot.RobotContainer;
import frc.robot.state.RobotStateEstimator;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;

/**
 * Heading controller drive command.
 */
public class TeleopDrive extends Command {
        public TeleopDrive(DoubleSupplier throttle, DoubleSupplier strafe, DoubleSupplier turn,
                        BooleanSupplier intakeLeft,
                        BooleanSupplier intakeRight, BooleanSupplier snap, BooleanSupplier slowDown) {
                mDrivetrain = RobotContainer.m_swerve;
                mThrottleSupplier = throttle;
                mStrafeSupplier = strafe;
                mTurnSupplier = turn;

                mLeftIntakeSupplier = intakeLeft;
                mRightIntakeSupplier = intakeRight;
                mSnapSupplier = snap;
                mSlowDownSupplier = slowDown;

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
        private BooleanSupplier mLeftIntakeSupplier, mRightIntakeSupplier, mSnapSupplier, mSlowDownSupplier;

        private SwerveRequest.FieldCentric driveNoHeading = new SwerveRequest.FieldCentric()
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);
        private SwerveRequest.FieldCentricFacingAngle driveWithHeading = new SwerveRequest.FieldCentricFacingAngle()
                        .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

        @Override
        public void initialize() {
        }

        @Override
        public void execute() {
                double throttle = mThrottleSupplier.getAsDouble()
                                * SwerveConstants.Kinematics.kTeleopLimits.maxLinearSpeed();
                double strafe = mStrafeSupplier.getAsDouble()
                                * SwerveConstants.Kinematics.kTeleopLimits.maxLinearSpeed();
                double turnFieldFrame = mTurnSupplier.getAsDouble()
                                * SwerveConstants.Kinematics.kTeleopLimits.maxOmega();
                double throttleFieldFrame = AllianceFlipUtil.shouldFlip() ? -throttle : throttle;
                double strafeFieldFrame = AllianceFlipUtil.shouldFlip() ? -strafe : strafe;

                boolean wantsLeftIntake = mLeftIntakeSupplier.getAsBoolean();
                boolean wantsRightIntake = mRightIntakeSupplier.getAsBoolean();
                boolean wantsSnap = mSnapSupplier.getAsBoolean();
                boolean wantsSlowDown = mSlowDownSupplier.getAsBoolean();

                if (wantsSlowDown) {
                        throttleFieldFrame *= 0.5; // slow down by 50% when scoring
                        strafeFieldFrame *= 0.5;
                }

                if (wantsSnap) {
                        Rotation2d headingSetpoint = Rotation2d.kZero;
                        
                        if (wantsSnap) {
                                double currentHeadingDegrees = RobotStateEstimator.getInstance().getEstimatedPose()
                                                .getRotation()
                                                .getDegrees();
                                headingSetpoint = Rotation2d
                                                .fromDegrees(60.0 * Math.round(currentHeadingDegrees / 60.0));
                        }
                        mDrivetrain.setControl(driveWithHeading
                                        .withVelocityX(throttleFieldFrame)
                                        .withVelocityY(strafeFieldFrame)
                                        .withTargetDirection(headingSetpoint));
                        Logger.recordOutput("DriveMaintainHeading/Mode", "Heading");
                        Logger.recordOutput("DriveMaintainHeading/HeadingSetpoint", headingSetpoint.getDegrees());
                } else {
                        mDrivetrain.setControl(driveNoHeading
                                        .withVelocityX(throttleFieldFrame)
                                        .withVelocityY(strafeFieldFrame)
                                        .withRotationalRate(turnFieldFrame));
                        Logger.recordOutput("DriveMaintainHeading/Mode", "NoHeading");
                }
        }
}