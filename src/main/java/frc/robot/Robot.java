// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.leds.Leds;
import frc.lib.team6328.VirtualSubsystem;

public class Robot extends LoggedRobot {
        private RobotContainer m_robotContainer;

        private Command m_autoCommand;
        private double m_autoStart;
        private boolean m_autoMessagePrinted;

        private final Timer m_canErrorTimer = new Timer();
        private final Timer m_disabledTimer = new Timer();

        private final Alert m_logReceiverQueueAlert = new Alert(
                        "Logging queue exceeded capacity, data will NOT be logged.",
                        AlertType.kError);
        private final Alert m_canErrorAlert = new Alert("CAN errors detected, robot may not be controllable.",
                        AlertType.kError);
        private final Alert m_lowBatteryAlert = new Alert(
                        "Battery voltage is very low, consider turning off the robot or replacing the battery.",
                        AlertType.kWarning);

        @Override
        public void robotInit() {
                Logger.recordMetadata("ProjectName", "LUCY-2025"); // Set a metadata value

                if (Constants.kIsReal) {
                        Logger.addDataReceiver(new WPILOGWriter()); // gotta plug a usb stick into rio 2 to have logging
                        Logger.addDataReceiver(new NT4Publisher());
                        LoggedPowerDistribution.getInstance(1, ModuleType.kRev);
                } else {
                        Logger.addDataReceiver(new NT4Publisher());
                }
                Logger.start();

                // Log active commands
                Map<String, Integer> commandCounts = new HashMap<>();
                BiConsumer<Command, Boolean> logCommandFunction = (Command command, Boolean active) -> {
                        String name = command.getName();
                        int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
                        commandCounts.put(name, count);
                        Logger.recordOutput(
                                        "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()),
                                        active);
                        Logger.recordOutput("CommandsAll/" + name, count > 0);
                };
                CommandScheduler.getInstance()
                                .onCommandInitialize(
                                                (Command command) -> {
                                                        logCommandFunction.accept(command, true);
                                                });
                CommandScheduler.getInstance()
                                .onCommandFinish(
                                                (Command command) -> {
                                                        logCommandFunction.accept(command, false);
                                                });
                CommandScheduler.getInstance()
                                .onCommandInterrupt(
                                                (Command command) -> {
                                                        logCommandFunction.accept(command, false);
                                                });

                // Default to blue alliance in sim
                if (!Constants.kIsReal) {
                        DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
                }

                // Start timers
                m_canErrorTimer.reset();
                m_canErrorTimer.start();
                m_disabledTimer.reset();
                m_disabledTimer.start();

                // Instantiate RobotContainer
                m_robotContainer = new RobotContainer();
        }

        @Override
        public void robotPeriodic() {
                Threads.setCurrentThreadPriority(true, 99);
                CommandScheduler.getInstance().run();
                VirtualSubsystem.periodicAll();

                // Print auto duration
                if (m_autoCommand != null) {
                        if (!m_autoCommand.isScheduled() && !m_autoMessagePrinted) {
                                if (DriverStation.isAutonomousEnabled()) {
                                        System.out.printf(
                                                        "*** Auto finished in %.2f secs ***%n",
                                                        Timer.getFPGATimestamp() - m_autoStart);
                                } else {
                                        System.out.printf(
                                                        "*** Auto cancelled in %.2f secs ***%n",
                                                        Timer.getFPGATimestamp() - m_autoStart);
                                }
                                m_autoMessagePrinted = true;
                                Leds.getInstance().autoFinished = true;
                                Leds.getInstance().autoFinishedTime = Timer.getFPGATimestamp();
                        }
                }

                // Check logging fault
                m_logReceiverQueueAlert.set(Logger.getReceiverQueueFault());

                // Update CAN error alert
                var canStatus = RobotController.getCANStatus();
                if (canStatus.receiveErrorCount > 0 || canStatus.transmitErrorCount > 0) {
                        m_canErrorTimer.reset();
                }
                m_canErrorAlert.set(!m_canErrorTimer.hasElapsed(0.5));

                // Update low battery alert
                if (DriverStation.isEnabled()) {
                        m_disabledTimer.reset();
                }
                if (RobotController.getBatteryVoltage() < 11.5 && m_disabledTimer.hasElapsed(2.0)) {
                        Leds.getInstance().lowBatteryAlert = true;
                        m_lowBatteryAlert.set(true);
                }

                // Robot container periodic methods
                m_robotContainer.checkControllers();

                Threads.setCurrentThreadPriority(true, 10);
        }

        @Override
        public void disabledInit() {
        }

        @Override
        public void disabledPeriodic() {
        }

        @Override
        public void disabledExit() {
        }

        @Override
        public void autonomousInit() {
                m_autoStart = Timer.getFPGATimestamp();
                m_autoMessagePrinted = false;
                m_autoCommand = RobotContainer.m_autoChooser.get();

                if (m_autoCommand != null) {
                        m_autoCommand.schedule();
                } else {
                        DriverStation.reportWarning("No Auto Rountine Selected!!!", false);
                }
        }

        @Override
        public void autonomousPeriodic() {
        }

        @Override
        public void autonomousExit() {
                if (m_autoCommand != null) {
                        m_autoCommand.cancel();
                }
        }

        @Override
        public void teleopInit() {
                if (m_autoCommand != null) {
                        m_autoCommand.cancel();
                }
/**
                if (!RobotContainer.m_elevator.isHomed()) {
                        RobotContainer.m_elevator.homingSequence().schedule();
                } */

                if (!RobotContainer.m_climb.isHomed()) {
                        RobotContainer.m_climb.homingSequence().schedule();
                }
                
        }

        @Override
        public void teleopPeriodic() {
        }

        @Override
        public void teleopExit() {
        }

        @Override
        public void testInit() {
                CommandScheduler.getInstance().cancelAll();
        }

        @Override
        public void testPeriodic() {
        }

        @Override
        public void testExit() {
        }

}