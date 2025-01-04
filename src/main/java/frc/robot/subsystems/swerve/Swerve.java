package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem
 * so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain implements Subsystem {

    private final SwerveRequest.ApplyChassisSpeeds autoRequest = new SwerveRequest.ApplyChassisSpeeds();

    public Swerve(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);

        tareEverything();
        configNeutralMode(NeutralModeValue.Brake);
        configurePathPlanner();
    }

    private void configurePathPlanner() {
        double driveBaseRadius = 0;
        for (var moduleLocation : m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        AutoBuilder.configureHolonomic(
                () -> this.getState().Pose, // Supplier of current robot pose
                this::seedFieldRelative, // Consumer for seeding pose against auto
                this::getCurrentRobotChassisSpeeds,
                (speeds) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer of ChassisSpeeds to drive the
                                                                             // robot
                new HolonomicPathFollowerConfig(
                        new PIDConstants(
                                SwerveConstants.PID.kTranslationkP,
                                SwerveConstants.PID.kTranslationkI,
                                SwerveConstants.PID.kTranslationkD),
                        new PIDConstants(
                                SwerveConstants.PID.kRotationkP,
                                SwerveConstants.PID.kRotationkI,
                                SwerveConstants.PID.kRotationkD),
                        SwerveConstants.Kinematics.kAutoLimits.maxLinearSpeed(),
                        driveBaseRadius,
                        new ReplanningConfig(),
                        0.004),
                () -> DriverStation.getAlliance().map(alliance -> alliance == DriverStation.Alliance.Red).orElse(false),
                this); // Subsystem for requirements
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    protected boolean checkIsOnCanFD(String name) {
        // Hack for CTRE's library sometimes saying the network is not CAN-FD
        return true;
    }

    @Override
    public void simulationPeriodic() {
        /* Assume 20ms update rate, get battery voltage from WPILib */
        updateSimState(0.02, RobotController.getBatteryVoltage());
        SmartDashboard.putString("Command", getCurrentCommand() == null ? "" : getCurrentCommand().getName());
    }

    /**
     * Gets the current robot-oriented chassis speeds
     * 
     * @return robot oriented chassis speeds
     */
    public ChassisSpeeds getCurrentRobotChassisSpeeds() {
        return getState().speeds;
    }

    /**
     * Gets the current field-oriented chassis speeds
     * 
     * @return field oriented chassis speeds
     */
    public Twist2d getCurrentFieldChassisSpeeds() {
        var state = getState();
        if (state == null || state.Pose == null) {
            return new Twist2d();
        }
        var robotAngle = state.Pose.getRotation();
        var chassisSpeeds = state.speeds;
        var fieldSpeeds = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
                .rotateBy(robotAngle);
        return new Twist2d(
                // TODO: Change to using pigeon2 omega
                fieldSpeeds.getX(), fieldSpeeds.getY(), chassisSpeeds.omegaRadiansPerSecond);
    }

}