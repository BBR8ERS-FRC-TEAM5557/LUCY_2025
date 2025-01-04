// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.team5557.factory.BurnManager;
import frc.lib.team5557.factory.SparkMaxFactory;
import frc.lib.team6328.Alert;
import frc.lib.team6328.Alert.AlertType;
import frc.robot.leds.Leds;
import frc.robot.state.vision.AprilTagVisionIO;
import frc.robot.state.vision.AprilTagVisionIOLimelight;
import frc.robot.state.vision.Vision;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.commands.TeleopDrive;
import frc.robot.util.AllianceFlipUtil;
//import frc.robot.util.FeedForwardCharacterization;
import frc.robot.util.FieldConstants;
import frc.robot.util.GeometryUtil;
import frc.robot.state.*;

import static frc.robot.Constants.*;
import static frc.robot.Constants.RobotMap.*;
import static frc.robot.state.vision.VisionConstants.*;

import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

public class RobotContainer {
  // create controller instances
  public static final CommandXboxController m_driver = new CommandXboxController(0);
  public static final CommandXboxController m_operator = new CommandXboxController(1);

  // create variables for physical subsystems
  public static Swerve m_swerve;
  public static Leds m_leds;

  // create variables for virtual subsystems
  public static Vision m_vision;
  public static RobotStateEstimator m_stateEstimator;

  // instantiate dashboard choosers / switches
  public static LoggedDashboardChooser<Command> m_autoChooser;

  // controller alerts
  private final Alert driverDisconnected = new Alert("Driver controller disconnected (port 0).",
      AlertType.WARNING);
  private final Alert operatorDisconnected = new Alert("Operator controller disconnected (port 1).",
      AlertType.WARNING);

  public RobotContainer() {
    // instantiate subsystems normally if the code is running irl (on a rio),
    // otherwise create simulation subsystems or blank subsystems (useful for taking
    // out entire subsystems in code without having to rewrite much)

    if (Constants.kIsReal) {
      m_swerve = new Swerve(
          SwerveConstants.TunerConstants.DrivetrainConstants,
          SwerveConstants.TunerConstants.FrontLeft,
          SwerveConstants.TunerConstants.FrontRight,
          SwerveConstants.TunerConstants.BackLeft,
          SwerveConstants.TunerConstants.BackRight);
      m_vision = new Vision(
          new AprilTagVisionIOLimelight(instanceNames[0], robotToCameraPoses[0]),
          new AprilTagVisionIOLimelight(instanceNames[1], robotToCameraPoses[1]));

    } else {
    }

    // Instantiate missing subsystems
    if (m_swerve == null) {
    }
    if (m_vision == null) {
      m_vision = new Vision(
          new AprilTagVisionIO() {
          },
          new AprilTagVisionIO() {
          });
    }

    m_stateEstimator = RobotStateEstimator.getInstance(); // get singleton
    m_leds = Leds.getInstance(); // get leds singleton
    m_autoChooser = new LoggedDashboardChooser<Command>("Driver/AutonomousChooser");

    // Alerts for constants
    if (Constants.kTuningMode) {
      SmartDashboard.putData("CommandScheduler", CommandScheduler.getInstance());
      new Alert("Tuning mode enabled, expect slower network", AlertType.INFO).set(true);
    }
    if (BurnManager.shouldBurn()) {
      new Alert("Burning flash enabled, consider disabling before competeing", AlertType.INFO).set(true);
    }

    configureBindings();
    generateEventMap();
    generateAutoChoices();
  }

  private void configureBindings() {
    // Bind driver and operator controls
    System.out.println("[Init] Binding controls");
    DriverStation.silenceJoystickConnectionWarning(true);

    /* SWERVING */
    TeleopDrive teleop = new TeleopDrive(this::getForwardInput, this::getStrafeInput, this::getRotationInput);
    m_swerve.setDefaultCommand(teleop.withName("Teleop Drive"));

    m_driver.start()
        .onTrue(Commands.runOnce(
            () -> m_stateEstimator.setPose(
                new Pose2d(
                    m_stateEstimator.getEstimatedPose()
                        .getTranslation(),
                    AllianceFlipUtil.apply(
                        new Rotation2d()))))
            .ignoringDisable(true).withName("ResetHeading"));

    PathPlannerPath path = PathPlannerPath.fromPathFile("alignAmp");
    PathConstraints constraints = new PathConstraints(
        4.0, 4.5,
        Units.degreesToRadians(540), Units.degreesToRadians(720));
    Command pathfindToAmp = AutoBuilder.pathfindThenFollowPath(
        path,
        constraints,
        0.0);
    // mDriver.rightBumper().and(mDriver.x()).whileTrue(pathfindToAmp);

    Command pulseControllers = Commands.sequence(Commands.runOnce(() -> {
      m_driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
      m_operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
    }).andThen(Commands.waitSeconds(0.25)), Commands.runOnce(() -> {
      m_driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
      m_operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    }).andThen(Commands.waitSeconds(0.25)), Commands.runOnce(() -> {
      m_driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
      m_operator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
    }).andThen(Commands.waitSeconds(0.25)), Commands.runOnce(() -> {
      m_driver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
      m_operator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
    })).withName("PulseControllers").ignoringDisable(true);

  }

  /** Updates the alerts for disconnected controllers. */
  public void checkControllers() {
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(m_driver.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(m_driver.getHID().getPort()));
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(m_operator.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(m_operator.getHID().getPort()));
  }

  private void generateAutoChoices() {
    System.out.println("[Init] Auto Routines");

    m_autoChooser.addDefaultOption("Do Nothing", null);
    m_autoChooser.addDefaultOption("Drive Out", AutoBuilder.buildAuto("DriveBack"));

    m_autoChooser.addDefaultOption("N3_S_C01", AutoBuilder.buildAuto("N3_S_C01"));

    // // Set up feedforward characterization
    // m_autoChooser.addOption(
    // "Drive FF Characterization",
    // new FeedForwardCharacterization(
    // m_swerve, m_swerve::runCharacterizationVolts,
    // m_swerve::getCharacterizationVelocity)
    // .finallyDo(m_swerve::stop));

    // m_autoChooser.addOption(
    // "Flywheels FF Characterization",
    // new FeedForwardCharacterization(
    // mFlywheels, mFlywheels::runCharacterizationVolts,
    // mFlywheels::getCharacterizationVelocity)
    // .finallyDo(mFlywheels::stop));
  }

  private void generateEventMap() {
  }

  public double getForwardInput() {
    return -square(deadband(m_driver.getLeftY(), 0.05));
  }

  public double getStrafeInput() {
    return -square(deadband(m_driver.getLeftX(), 0.05));
  }

  public double getRotationInput() {
    double leftTrigger = square(deadband(m_driver.getLeftTriggerAxis(), 0.05));
    double rightTrigger = square(deadband(m_driver.getRightTriggerAxis(), 0.05));

    // return -square(deadband(mDriver.getRightX(), 0.05));

    return leftTrigger > rightTrigger ? leftTrigger : -rightTrigger;
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