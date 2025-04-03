// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team6328.AllianceFlipUtil;
import frc.robot.leds.Leds;
import frc.robot.state.vision.AprilTagVisionIO;
import frc.robot.state.vision.Vision;
import frc.robot.subsystems.SuperstructureFactory;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOTalonFX;
import frc.robot.subsystems.flywheels.Flywheels;
import frc.robot.subsystems.flywheels.FlywheelsIO;
import frc.robot.subsystems.flywheels.FlywheelsIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOTalonFX;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.commands.TeleopDrive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.rollers.Rollers;
import frc.robot.subsystems.rollers.RollersIO;
import frc.robot.subsystems.rollers.RollersIOTalonFX;
import frc.robot.state.*;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

public class RobotContainer {
        // create controller instances
        public static final CommandXboxController m_driver = new CommandXboxController(0);

        // create variables for physical subsystems
        public static Swerve m_swerve;
        public static Elevator m_elevator;
        public static Flywheels m_flywheels;
        public static Climb m_climb;
        public static Orchestra m_orchestra;

        // create variables for virtual subsystems
        public static Vision m_vision;
        public static RobotStateEstimator m_stateEstimator;
        public static Leds m_leds;
        

        // instantiate dashboard choosers / switches
        public static LoggedDashboardChooser<Command> m_autoChooser;

        // controller alerts
        private final Alert driverDisconnected = new Alert("Driver controller disconnected (port 0).",
                        AlertType.kWarning);
        private final Alert tuningMode = new Alert("Tuning mode enabled, expect slower network", AlertType.kInfo);

        public RobotContainer() {
                System.out.println("[Init] Instantiating RobotContainer");
                // instantiate subsystems normally if the code is running irl (on a rio),
                // otherwise create simulation subsystems or blank subsystems (useful for taking
                // out entire subsystems in code without having to rewrite much)

                m_swerve = new Swerve(
                                SwerveConstants.TunerConstants.DrivetrainConstants,
                                SwerveConstants.TunerConstants.FrontLeft,
                                SwerveConstants.TunerConstants.FrontRight,
                                SwerveConstants.TunerConstants.BackLeft,
                                SwerveConstants.TunerConstants.BackRight);

                
                m_elevator = new Elevator(
                                new ElevatorIOTalonFX());  


                m_flywheels = new Flywheels(
                                new FlywheelsIOTalonFX());   
                
              

                                /**
                m_climb = new Climb(
                                new ClimbIOTalonFX()); */

                // Instantiate missing subsystems
                if (m_elevator == null) {
                        m_elevator = new Elevator(new ElevatorIO() {
                        });
                }

                

                if (m_flywheels == null) {
                        m_flywheels = new Flywheels(new FlywheelsIO() {
                        });
                }


                if (m_climb == null) {
                        m_climb = new Climb(new ClimbIO() {
                        });
                }

               

                if (m_vision == null) {
                        m_vision = new Vision(
                                        new AprilTagVisionIO() {
                                        },
                                        new AprilTagVisionIO() {
                                        });
                }

                m_stateEstimator = RobotStateEstimator.getInstance(); // get state estimator singleton
                m_leds = Leds.getInstance(); // get leds singleton
                m_autoChooser = new LoggedDashboardChooser<Command>("Driver/AutonomousChooser");

                // Alerts for constants
                tuningMode.set(true);
                if (Constants.kTuningMode) {
                        SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
                        tuningMode.set(true);
                }

                configureBindings();
                generateEventMap();
                generateAutoChoices();
        }

        private void configureBindings() {
                // Bind driver and operator controls
                System.out.println("[Init] Binding controls");
                DriverStation.silenceJoystickConnectionWarning(true);

                /* DRIVING */
                // TELEOP DRIVE
                TeleopDrive teleop = new TeleopDrive(
                                this::getForwardInput,
                                this::getStrafeInput,
                                this::getRotationInput,
                                this::getLeftIntakeInput,
                                this::getRightIntakeInput,
                                this::getSnapInput,
                                this::getSlowDownInput);
                m_swerve.setDefaultCommand(teleop.withName("TeleopDrive"));

               // m_orchestra.addInstrument(m_intake);

                /* UTIL */
                // ZERO SWERVE
                m_driver.start().onTrue(Commands.runOnce(
                                () -> m_stateEstimator.setPose(
                                                new Pose2d(
                                                                m_stateEstimator.getEstimatedPose().getTranslation(),
                                                                AllianceFlipUtil.apply(new Rotation2d()))))
                                .ignoringDisable(true)
                                .withName("ResetHeading"));

                // HOME SUPERSTRUCTURE

                /**
                m_driver.back().whileTrue(Commands
                                .parallel(
                                                m_elevator.homingSequence())
                                .withName("HomeSuperstructure")); */

                // COAST MODE
                m_driver.back().whileTrue(
                                Commands.startEnd(
                                                () -> {
                                                        m_swerve.setBrakeMode(false);
                                                        m_elevator.setBrakeMode(false);
                                                        m_flywheels.setBrakeMode(false);
                                                        m_climb.setBrakeMode(false);
                                                },
                                                () -> {
                                                        m_swerve.setBrakeMode(true);
                                                        m_elevator.setBrakeMode(true);
                                                        m_flywheels.setBrakeMode(true);
                                                        m_climb.setBrakeMode(true);
                                                },
                                                m_swerve, m_elevator, m_flywheels, m_climb)
                                                .unless(() -> DriverStation.isEnabled())
                                                .ignoringDisable(true)
                                                .withName("RobotGoLimp"));

                // ADJUST SCORING LEVEL
                m_driver.povUp().onTrue(SuperstructureFactory.adjustLevel(1, 0.75));
                m_driver.povDown().onTrue(SuperstructureFactory.adjustLevel(-1, 0.75));
                m_driver.a().onTrue(SuperstructureFactory.adjustLevel(0, 0.0));

                /* DRIVER CONTROLS */
                // SUPERSTRUCTURE SCORE
                m_driver.leftTrigger().whileTrue(SuperstructureFactory.scoreCoral().finallyDo(() -> {
                        SuperstructureFactory.stow().schedule();
                }));

        
                m_driver.b().onTrue(m_elevator.zeroPosition());

                // FLYWHEELS SCORE
                m_driver.rightTrigger().whileTrue(m_flywheels.scoreCoral());

                 
                // INTAKE
                Trigger intakeTrigger = m_driver.leftBumper().or(m_driver.rightBumper());
                intakeTrigger.whileTrue(
                                SuperstructureFactory.intakeCoral().withDeadline(m_flywheels.intakeCoral())
                                                .finallyDo(() -> {
                                                        SuperstructureFactory.stow().schedule();
                                                })); 

                // POP ALGAE
                m_driver.x().whileTrue(SuperstructureFactory.executePopAlgae()
                                .finallyDo(() -> {
                                        SuperstructureFactory.stow().schedule();
                                }));

                m_driver.y().whileTrue(SuperstructureFactory.intakeCoral().alongWith(m_flywheels.intakeCoral()));

                // CLIMB
               // m_driver.b().onTrue(SuperstructureFactory.toggleClimb());
                m_driver.povLeft().whileTrue(m_climb.runVoltageCommand(() -> -12.0));
                m_driver.povRight().whileTrue(m_climb.runVoltageCommand(() -> 12.0));

                /* ENDGAME ALERTS */
                new Trigger(
                                () -> DriverStation.isTeleopEnabled()
                                                && DriverStation.getMatchTime() > 0
                                                && DriverStation.getMatchTime() <= Math.round(30.0))
                                .onTrue(controllerRumbleCommand()
                                                .withTimeout(0.5)
                                                .beforeStarting(() -> m_leds.endgameAlert = true)
                                                .finallyDo(() -> m_leds.endgameAlert = false));
                new Trigger(
                                () -> DriverStation.isTeleopEnabled()
                                                && DriverStation.getMatchTime() > 0
                                                && DriverStation.getMatchTime() <= Math.round(15.0))
                                .onTrue(controllerRumbleCommand()
                                                .withTimeout(0.2)
                                                .andThen(Commands.waitSeconds(0.1))
                                                .repeatedly()
                                                .withTimeout(0.9)
                                                .beforeStarting(() -> m_leds.endgameAlert = true)
                                                .finallyDo(() -> m_leds.endgameAlert = false)); // Rumble
                                                                                                // three
                                                                                                // times
        }

        /** Updates the alerts for disconnected controllers. */
        public void checkControllers() {
                driverDisconnected.set(
                                !DriverStation.isJoystickConnected(m_driver.getHID().getPort())
                                                || !DriverStation.getJoystickIsXbox(m_driver.getHID().getPort()));
        }

        private void generateAutoChoices() {
                System.out.println("[Init] Auto Routines");

                m_autoChooser.addDefaultOption("Do Nothing", null);

                m_autoChooser.addDefaultOption("drive_back_left", AutoBuilder.buildAuto("drive_back_left"));

                m_autoChooser.addDefaultOption("just_paths", AutoBuilder.buildAuto("just_paths"));

                m_autoChooser.addDefaultOption("drive_back_right", AutoBuilder.buildAuto("drive_back_right"));

                m_autoChooser.addDefaultOption("CA5-C3-C6-TRY THIS ONE!!", AutoBuilder.buildAuto("CA5-C3-C6-C5"));

                m_autoChooser.addDefaultOption("CA5_C3_C6-paths", AutoBuilder.buildAuto("CA5_C3_C6-paths"));

                m_autoChooser.addDefaultOption("CA5_C3_C6-real", AutoBuilder.buildAuto("CA5_C3_C6-real"));

                m_autoChooser.addDefaultOption("CA2-C12-C9-C10", AutoBuilder.buildAuto("CA2-C12-C9-C10"));
        }

        private void generateEventMap() {
                NamedCommands.registerCommand("scoreL4",
                                Commands.print("scoring L4").alongWith(
                                                SuperstructureFactory.scoreL4Coral()
                                                                .withDeadline(SuperstructureFactory
                                                                                .waitUntilAtSetpoint()
                                                                                .andThen(Commands.waitSeconds(1))
                                                                                .andThen(m_flywheels.scoreCoral()
                                                                                                .withTimeout(1.5))))
                                                .finallyDo(() -> {
                                                        SuperstructureFactory.stow().schedule();
                                                }));

                NamedCommands.registerCommand("prepL4",
                                Commands.print("prep L4")
                                                .alongWith(SuperstructureFactory.scoreL4Coral()));

             

                NamedCommands.registerCommand("scoreL3",
                                Commands.print("scoring L3")
                                                .alongWith(SuperstructureFactory.scoreL3Coral())
                                                .withDeadline(SuperstructureFactory.waitUntilAtSetpoint()
                                                                .andThen(Commands.waitSeconds(0.25))
                                                                .andThen(m_flywheels.scoreCoral().withTimeout(1.5)))
                                                .finallyDo(() -> {
                                                        SuperstructureFactory.stow().schedule();
                                                }));

                NamedCommands.registerCommand("scoreL2",
                                Commands.print("scoring L2")
                                                .alongWith(SuperstructureFactory.scoreL2Coral())
                                                .withDeadline(SuperstructureFactory.waitUntilAtSetpoint()
                                                                .andThen(Commands.waitSeconds(0.25))
                                                                .andThen(m_flywheels.scoreCoral().withTimeout(1.5)))
                                                .finallyDo(() -> {
                                                        SuperstructureFactory.stow().schedule();
                                                }));

                NamedCommands.registerCommand("scoreL1",
                                Commands.print("scoring L1")
                                                .alongWith(SuperstructureFactory.scoreL1Coral())
                                                .withDeadline(SuperstructureFactory.waitUntilAtSetpoint()
                                                                .andThen(Commands.waitSeconds(0.25))
                                                                .andThen(m_flywheels.scoreCoral().withTimeout(1.5)))
                                                .finallyDo(() -> {
                                                        SuperstructureFactory.stow().schedule();
                                                }));

                NamedCommands.registerCommand("intakeCoral",
                                Commands.print("Intaking coral")
                                                .alongWith(SuperstructureFactory.intakeCoral())
                                                .withDeadline(m_flywheels.intakeCoral())
                                                .finallyDo(() -> {
                                                        SuperstructureFactory.stow();
                                                }));

                NamedCommands.registerCommand("shootCoral",
                                                Commands.print("shooooooting coral")
                                                                .alongWith(m_flywheels.scoreCoral().withTimeout(1))
                                                                
                                                                .andThen(SuperstructureFactory.stow().withTimeout(1.5)));
                
        }

        // Creates controller rumble command
        public static Command controllerRumbleCommand() {
                return Commands.startEnd(
                                () -> {
                                        m_driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
                                },
                                () -> {
                                        m_driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
                                });
        }

        public double getForwardInput() {
                return -square(deadband(m_driver.getLeftY(), 0.1));
        }

        public double getStrafeInput() {
                return -square(deadband(m_driver.getLeftX(), 0.1));
        }

        public double getRotationInput() {
                return -square(deadband(m_driver.getRightX(), 0.1));
        }

        public boolean getLeftIntakeInput() {
                return m_driver.leftBumper().getAsBoolean();
        }

        public boolean getRightIntakeInput() {
                return m_driver.rightBumper().getAsBoolean();
        }

        public boolean getSnapInput() {
                return m_driver.a().getAsBoolean() || m_driver.x().getAsBoolean();
        }

        public boolean getSlowDownInput() {
                return m_driver.leftTrigger().getAsBoolean();
        }

        private static double deadband(double value, double tolerance) {
                if (Math.abs(value) < tolerance)
                        return 0.0;
                return Math.copySign(value, (value - tolerance) / (1.0 - tolerance));
        }

        public static double square(double value) {
                return Math.copySign(value * value, value);
        }

        public static double cube(double value) {
                return Math.copySign(value * value * value, value);
        }
}