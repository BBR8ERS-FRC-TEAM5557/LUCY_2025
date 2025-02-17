package frc.robot.subsystems.swerve.commands;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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
public class DriveToPose extends Command {
    public DriveToPose(DoubleSupplier throttle, DoubleSupplier strafe) {
        mDrivetrain = RobotContainer.m_swerve;
        mThrottleSupplier = throttle;
        mStrafeSupplier = strafe;
        mThrottlePIDController = new PIDController(SwerveConstants.PID.kTranslationkP, SwerveConstants.PID.kTranslationkI, SwerveConstants.PID.kTranslationkD);
        mStrafePIDController = new PIDController(SwerveConstants.PID.kTranslationkP, SwerveConstants.PID.kTranslationkI, SwerveConstants.PID.kTranslationkD);

        driveWithHeading.HeadingController.setPID(
                SwerveConstants.PID.kRotationkP,
                SwerveConstants.PID.kRotationkI,
                SwerveConstants.PID.kRotationkD);
        driveWithHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(mDrivetrain);
        setName("Teleop Drive To Pose");

        if (Robot.isSimulation()) {
            driveWithHeading.DriveRequestType = SwerveModule.DriveRequestType.OpenLoopVoltage;
        }
    }

    private Swerve mDrivetrain;
    private PIDController mThrottlePIDController, mStrafePIDController;
    private DoubleSupplier mThrottleSupplier, mStrafeSupplier;
    private Optional<Pose2d> mPoseSetpoint = Optional.empty();

    private double kMaxAdjustmentSpeedMPS = 0.25;
    private double kAllowableErrorMeters = 0.25;

    private SwerveRequest.FieldCentricFacingAngle driveWithHeading = new SwerveRequest.FieldCentricFacingAngle()
            .withDriveRequestType(SwerveModule.DriveRequestType.Velocity);

    @Override
    public void initialize() {
        mPoseSetpoint = Optional.empty();
        mThrottlePIDController.reset();
        mStrafePIDController.reset();
        mThrottlePIDController.setTolerance(kAllowableErrorMeters);
        mStrafePIDController.setTolerance(kAllowableErrorMeters);
    }

    @Override
    public void execute() {
        Pose2d currentPose = RobotStateEstimator.getInstance().getEstimatedPose();

        if (mPoseSetpoint.isEmpty()) {
            //TODO: add logic for getting nearest reef setpoint
            //TODO: add logic for transform left/right of generic midpoint
            //mPoseSetpoint = Optional.of(RobotStateEstimator.getInstance().getEstimatedPose().getRotation());
        }

        if (mThrottlePIDController.atSetpoint() && mStrafePIDController.atSetpoint()) {
            double throttlePosAdjustment = mThrottleSupplier.getAsDouble() * kMaxAdjustmentSpeedMPS * Robot.defaultPeriodSecs;
            double strafePosAdjustment = mStrafeSupplier.getAsDouble() * kMaxAdjustmentSpeedMPS * Robot.defaultPeriodSecs;
            double throttleFieldFrame = AllianceFlipUtil.shouldFlip() ? -throttlePosAdjustment : throttlePosAdjustment;
            double strafeFieldFrame = AllianceFlipUtil.shouldFlip() ? -strafePosAdjustment : strafePosAdjustment;

            Transform2d poseAdjustment = new Transform2d(throttleFieldFrame, strafeFieldFrame, new Rotation2d());
            mPoseSetpoint.get().plus(poseAdjustment);
        }

        double throttleSetpoint = Util.clamp(
            mThrottlePIDController.calculate(currentPose.getX(), mPoseSetpoint.get().getX()), 
            -SwerveConstants.Kinematics.kTeleopLimits.maxLinearSpeed(), 
            SwerveConstants.Kinematics.kTeleopLimits.maxLinearSpeed());

        double strafeSetpoint = Util.clamp(
            mStrafePIDController.calculate(currentPose.getY(), mPoseSetpoint.get().getY()),
            -SwerveConstants.Kinematics.kTeleopLimits.maxLinearSpeed(), 
            SwerveConstants.Kinematics.kTeleopLimits.maxLinearSpeed());

        mDrivetrain.setControl(driveWithHeading.withVelocityX(throttleSetpoint).withVelocityY(strafeSetpoint)
                .withTargetDirection(mPoseSetpoint.get().getRotation()));

        Logger.recordOutput("DriveToPose/PoseSetpoint", mPoseSetpoint.get());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
