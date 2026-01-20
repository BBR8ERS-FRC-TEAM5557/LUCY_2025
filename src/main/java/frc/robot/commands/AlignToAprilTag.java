package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
// import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Constants.VisionConstants;
// import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.VisionSubsystem;

public class AlignToAprilTag extends Command {
    private final Swerve m_swerve;
    private final VisionSubsystem m_visionSubsystem;
    private final PIDController m_rotationController;

    /**
     * Creates a command that rotates the robot to face the detected AprilTag
     * @param Swerve The drive subsystem
     * @param visionSubsystem The vision subsystem
     */
    public AlignToAprilTag(Swerve swerve, VisionSubsystem visionSubsystem) {
        m_swerve = swerve;
        m_visionSubsystem = visionSubsystem;

        // Create PID controller for rotation alignment
        m_rotationController = new PIDController(
            VisionConstants.kAlignP,
            VisionConstants.kAlignI,
            VisionConstants.kAlignD);

        // Set tolerance for alignment
        m_rotationController.setTolerance(
            VisionConstants.kAlignPositionTolerance,
            VisionConstants.kAlignVelocityTolerance);

        // Enable continuous input for angles (-180 to 180 degrees)
        m_rotationController.enableContinuousInput(-180.0, 180.0);

        // addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        m_rotationController.reset();
        Logger.recordOutput("Commands/AlignToAprilTag/Running", true);
    }

    @Override
    public void execute() {
        if (m_visionSubsystem.hasTargets()) {
            // Get the yaw angle to the target (positive = target is to the right)
            double targetYaw = m_visionSubsystem.getBestTargetYaw();

            // Calculate rotation speed using PID controller
            // We want to drive targetYaw to 0 (robot facing the target)
            double rotationSpeed = m_rotationController.calculate(targetYaw, 0.0);

            // Limit rotation speed
            rotationSpeed = Math.max(-VisionConstants.kMaxAlignAngularVelocity,
                                   Math.min(VisionConstants.kMaxAlignAngularVelocity, rotationSpeed));

            // Create chassis speeds with only rotation (no translation)
            ChassisSpeeds speeds = new ChassisSpeeds(0.0, 0.0, rotationSpeed);

            // Drive the robot
            m_swerve.drive(speeds);

            Logger.recordOutput("Commands/AlignToAprilTag/TargetYaw", targetYaw);
            Logger.recordOutput("Commands/AlignToAprilTag/RotationSpeed", rotationSpeed);
            Logger.recordOutput("Commands/AlignToAprilTag/AtSetpoint", m_rotationController.atSetpoint());
        } else {
            // No targets visible, stop rotating
            m_swerve.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
            Logger.recordOutput("Commands/AlignToAprilTag/NoTargets", true);
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Stop the robot when command ends
        m_swerve.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        Logger.recordOutput("Commands/AlignToAprilTag/Running", false);
    }

    @Override
    public boolean isFinished() {
        // Command finishes when aligned to target or no targets are visible
        return !m_visionSubsystem.hasTargets() || m_rotationController.atSetpoint();
    }
}