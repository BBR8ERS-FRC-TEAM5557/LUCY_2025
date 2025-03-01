// Copyright (c) 2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.team6328.AllianceFlipUtil;
import frc.lib.team6328.GeomUtil;
import frc.lib.team6328.LoggedTunableNumber;
import frc.robot.Constants;
import frc.robot.leds.Leds;
import frc.robot.state.RobotStateEstimator;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.util.FieldConstants.CoralObjective;
import frc.robot.util.FieldConstants.Reef;
import frc.robot.util.FieldConstants.ReefLevel;

import java.util.function.*;

public class AutoScore {
        // Radius of regular hexagon is side length
        private static final LoggedTunableNumber maxDistanceReefLineup = new LoggedTunableNumber(
                        "AutoScore/MaxDistanceReefLineup", 1.5);
        public static final LoggedTunableNumber minDistanceReefClearAlgae = new LoggedTunableNumber(
                        "AutoScore/MinDistanceReefClearAlgae", Units.inchesToMeters(18.0));
        public static final LoggedTunableNumber minDistanceReefClear = new LoggedTunableNumber(
                        "AutoScore/MinDistanceReefClear", Units.inchesToMeters(12.0));

        private static final LoggedTunableNumber l1AlignOffsetX = new LoggedTunableNumber("AutoScore/L1AlignOffsetX",
                        0.5);
        private static final LoggedTunableNumber l1AlignOffsetY = new LoggedTunableNumber("AutoScore/L1AlignOffsetY",
                        0.0);
        private static final LoggedTunableNumber l1AlignOffsetDegrees = new LoggedTunableNumber(
                        "AutoScore/L1AlignOffsetDegrees", 20);

        private static final LoggedTunableNumber l2AlignOffsetX = new LoggedTunableNumber("AutoScore/L2AlignOffsetX",
                        0.3);
        private static final LoggedTunableNumber l3AlignOffsetX = new LoggedTunableNumber("AutoScore/L3AlignOffsetX",
                        0.3);
        private static final LoggedTunableNumber l4AlignOffsetX = new LoggedTunableNumber(
                        "AutoScore/L4AlignOffsetX",
                        0.3);

        public static Command getAutoDriveCommand(
                        Swerve drive,
                        Supplier<ReefLevel> reefLevel,
                        DoubleSupplier driverThrottle,
                        DoubleSupplier driverStrafe,
                        boolean shouldEnd) {
                Command driveCommand = new DriveToPose(
                                drive,
                                () -> {
                                        CoralObjective objective = getNearestCoralObjective(reefLevel.get());
                                        if (reefLevel.get() == ReefLevel.L1) {
                                                return getDriveTarget(AllianceFlipUtil.apply(getL1Pose(objective)));
                                        }
                                        return getDriveTarget(AllianceFlipUtil.apply(getCoralScorePose(objective)));
                                },
                                () -> {
                                        double scalar = SwerveConstants.Kinematics.kTeleopLimits.maxLinearSpeed();
                                        Translation2d linearFF = new Translation2d(
                                                        driverThrottle.getAsDouble() * scalar,
                                                        driverStrafe.getAsDouble() * scalar)
                                                        .times(AllianceFlipUtil.shouldFlip() ? -1.0 : 1.0);

                                        return linearFF;
                                },
                                () -> 0.0,
                                shouldEnd);

                return Commands.runOnce(
                                () -> {
                                        // Start LEDs
                                        Leds.getInstance().autoDrive = true;
                                })
                                .andThen(driveCommand)
                                .finallyDo(
                                                interrupted -> {
                                                        Leds.getInstance().autoDrive = false;
                                                });
        }

        public static Command getAutoDriveCommand(
                        Swerve drive,
                        Supplier<ReefLevel> reefLevel) {
                return getAutoDriveCommand(
                                drive,
                                reefLevel,
                                () -> 0.0,
                                () -> 0.0,
                                true);
        }

        /** Get drive target. */
        public static Pose2d getDriveTarget(Pose2d goal) {
                Pose2d robot = RobotStateEstimator.getInstance().getEstimatedPose();
                var offset = robot.relativeTo(goal);
                double yDistance = Math.abs(offset.getY());
                double xDistance = Math.abs(offset.getX());
                double shiftXT = MathUtil.clamp(
                                (yDistance / (Reef.faceLength * 2)) + ((xDistance - 0.3) / (Reef.faceLength * 3)),
                                0.0,
                                1.0);
                double shiftYT = MathUtil.clamp(yDistance <= 0.2 ? 0.0 : offset.getX() / Reef.faceLength, 0.0, 1.0);
                return goal.transformBy(
                                GeomUtil.toTransform2d(
                                                -shiftXT * maxDistanceReefLineup.get(),
                                                Math.copySign(shiftYT * maxDistanceReefLineup.get() * 0.8,
                                                                offset.getY())));
        }

        public static CoralObjective getNearestCoralObjective(ReefLevel level) {
                double nearestDistance = Double.MAX_VALUE;
                int nearestBranchID = 0;
                Translation2d robot = AllianceFlipUtil
                                .apply(RobotStateEstimator.getInstance().getEstimatedPose().getTranslation());

                for (int branchID = 0; branchID < Reef.branchPositions2d.size(); branchID++) {
                        Translation2d branch = Reef.branchPositions2d.get(branchID).get(level).getTranslation();
                        double distance = branch.getDistance(robot);
                        if (distance < nearestDistance) {
                                nearestBranchID = branchID;
                                nearestDistance = distance;
                        }
                }

                return new CoralObjective(nearestBranchID, level);
        }

        public static Pose2d getBranchPose(CoralObjective objective) {
                return Reef.branchPositions2d.get(objective.branchId()).get(objective.reefLevel());
        }

        /** Get position of robot aligned with branch for selected objective. */
        public static Pose2d getCoralScorePose(CoralObjective coralObjective) {
                Pose2d branchPose = getBranchPose(coralObjective);
                double offsetX = Constants.Physical.kRobotLength / 2.0;
                if (coralObjective.reefLevel() == ReefLevel.L2) {
                        offsetX += l2AlignOffsetX.get();
                }
                if (coralObjective.reefLevel() == ReefLevel.L3) {
                        offsetX += l3AlignOffsetX.get();
                }
                if (coralObjective.reefLevel() == ReefLevel.L4) {
                        offsetX += l4AlignOffsetX.get();
                }

                return branchPose.transformBy(
                                new Transform2d(
                                                offsetX,
                                                0.0,
                                                Rotation2d.k180deg));
        }

        private static Pose2d getL1Pose(CoralObjective coralObjective) {
                int face = coralObjective.branchId() / 2;
                double offsetX = (Constants.Physical.kRobotLength / 2.0) + l1AlignOffsetX.get();
                return Reef.centerFaces[face].transformBy(
                                new Transform2d(
                                                offsetX,
                                                l1AlignOffsetY.get(),
                                                Rotation2d.fromDegrees(l1AlignOffsetDegrees.get())));
        }
}