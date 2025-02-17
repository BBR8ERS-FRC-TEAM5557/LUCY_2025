package frc.robot.subsystems.swerve;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static frc.lib.team6328.PhoenixUtil.tryUntilOk;
import frc.robot.util.AllianceFlipUtil;
import frc.robot.util.Util;
import static frc.robot.subsystems.swerve.SwerveConstants.*;
import static frc.robot.Constants.Physical.*;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class Swerve extends SwerveDrivetrain<TalonFX, TalonFX, CANcoder> implements Subsystem {

        private final SwerveRequest.ApplyRobotSpeeds autoRequest = new SwerveRequest.ApplyRobotSpeeds();

        @SafeVarargs
        public Swerve(SwerveDrivetrainConstants driveTrainConstants,
                        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>... modules) {
                super(TalonFX::new, TalonFX::new, CANcoder::new, driveTrainConstants, modules);
                System.out.println("[Init] Instantiating Swerve");
                tareEverything();
                seedFieldCentric();
                configNeutralMode(NeutralModeValue.Brake);
                configurePathPlanner();
                configureDashboard();
        }

        private void configurePathPlanner() {
                ModuleConfig kModuleConfig = new ModuleConfig(
                                kWheelRadiusMeters,
                                kTrueMaxSpeed,
                                kWheelCOF,
                                DCMotor.getKrakenX60(1),
                                kDriveGearRatio,
                                80,
                                1);
                RobotConfig kRobotConfig = new RobotConfig(
                                kMass,
                                kMOI,
                                kModuleConfig,
                                getModuleLocations());

                AutoBuilder.configure(
                                () -> this.getState().Pose, // Supplier of current robot pose
                                (pose) -> resetPose(pose), // Consumer for seeding pose against auto
                                () -> getCurrentRobotChassisSpeeds(),
                                (speeds, feedforwards) -> this.setControl(autoRequest.withSpeeds(speeds)), // Consumer
                                                                                                           // of robot
                                                                                                           // speeds
                                new PPHolonomicDriveController(
                                                new PIDConstants(
                                                                SwerveConstants.PID.kTranslationkP,
                                                                SwerveConstants.PID.kTranslationkI,
                                                                SwerveConstants.PID.kTranslationkD),
                                                new PIDConstants(
                                                                SwerveConstants.PID.kRotationkP,
                                                                SwerveConstants.PID.kRotationkI,
                                                                SwerveConstants.PID.kRotationkD),
                                                0.004),
                                kRobotConfig,
                                () -> AllianceFlipUtil.shouldFlip(),
                                this); // Subsystem for requirements
        }

        private void configureDashboard() {
                ShuffleboardTab shuffleboardTab = Shuffleboard
                                .getTab("Swerve");
                shuffleboardTab.addNumber("Heading", () -> Util.truncate(getState().Pose.getRotation().getDegrees(), 2))
                                .withWidget(BuiltInWidgets.kGyro);
                shuffleboardTab
                                .addNumber(
                                                "Velocity",
                                                () -> Util.truncate(
                                                                Math.hypot(getState().Speeds.vxMetersPerSecond,
                                                                                getState().Speeds.vyMetersPerSecond),
                                                                2))
                                .withWidget(BuiltInWidgets.kGraph);

                ShuffleboardLayout containerFL = shuffleboardTab.getLayout("FL Module", BuiltInLayouts.kList)
                                .withSize(2, 2)
                                .withPosition(0, 0);
                containerFL.addNumber("Current Velocity", () -> getModule(0).getCurrentState().speedMetersPerSecond);
                containerFL.addNumber("Current Angle (Deg)", () -> getModule(0).getCurrentState().angle.getDegrees());

                ShuffleboardLayout containerFR = shuffleboardTab.getLayout("FR Module", BuiltInLayouts.kList)
                                .withSize(2, 2)
                                .withPosition(2, 0);
                containerFR.addNumber("Current Velocity", () -> getModule(1).getCurrentState().speedMetersPerSecond);
                containerFR.addNumber("Current Angle (Deg)", () -> getModule(1).getCurrentState().angle.getDegrees());

                ShuffleboardLayout containerBL = shuffleboardTab.getLayout("BL Module", BuiltInLayouts.kList)
                                .withSize(2, 2)
                                .withPosition(2, 0);
                containerBL.addNumber("Current Velocity", () -> getModule(2).getCurrentState().speedMetersPerSecond);
                containerBL.addNumber("Current Angle (Deg)", () -> getModule(2).getCurrentState().angle.getDegrees());

                ShuffleboardLayout containerBR = shuffleboardTab.getLayout("BR Module", BuiltInLayouts.kList)
                                .withSize(2, 2)
                                .withPosition(2, 2);
                containerBR.addNumber("Current Velocity", () -> getModule(3).getCurrentState().speedMetersPerSecond);
                containerBR.addNumber("Current Angle (Deg)", () -> getModule(3).getCurrentState().angle.getDegrees());

        }

        public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
                return run(() -> this.setControl(requestSupplier.get()));
        }

        public void setBrakeMode(boolean enabled) {
                tryUntilOk(5, () -> configNeutralMode(enabled ? NeutralModeValue.Brake : NeutralModeValue.Coast));
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
                return getState().Speeds;
        }

        /**
         * Gets the current field-oriented chassis speeds
         * 
         * @return field oriented chassis speeds
         */
        @AutoLogOutput(key = "Swerve/FieldVelocity")
        public ChassisSpeeds getCurrentFieldChassisSpeeds() {
                var state = getState();
                if (state == null || state.Pose == null) {
                        return new ChassisSpeeds();
                }
                var robotAngle = state.Pose.getRotation();
                var chassisSpeeds = state.Speeds;
                var fieldSpeeds = new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
                                .rotateBy(robotAngle);
                return new ChassisSpeeds(
                                // TODO: Change to using pigeon2 omega
                                fieldSpeeds.getX(), fieldSpeeds.getY(),
                                getPigeon2().getAngularVelocityZWorld().getValueAsDouble());
        }
}