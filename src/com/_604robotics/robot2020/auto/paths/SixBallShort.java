package com._604robotics.robot2020.auto.paths;

import com._604robotics.robot2020.Robot2020;
import com._604robotics.robot2020.auto.SparkTrajectoryTracker;
import com._604robotics.robot2020.auto.macros.IntakeMacro;
import com._604robotics.robot2020.auto.macros.ShootSequence;
import com._604robotics.robot2020.constants.Calibration;
import com._604robotics.robotnik.prefabs.auto.TrajectoryCreator;
import com._604robotics.robotnik.prefabs.coordinators.ParallelRaceCoordinator;
import com._604robotics.robotnik.prefabs.coordinators.StatefulCoordinator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import java.util.List;

public class SixBallShort extends StatefulCoordinator {
  public SixBallShort(Robot2020 robot, TrajectoryCreator trajectoryCreator) {
    super(SixBallShort.class);

    addState(
        "Drive Past Auto Line",
        new SparkTrajectoryTracker(
            trajectoryCreator.getTrajectory(
                List.of(
                    new Pose2d(
                        Units.feetToMeters(9.3),
                        Units.feetToMeters(20.0),
                        Rotation2d.fromDegrees(-180)),
                    new Pose2d(
                        Units.feetToMeters(15.3),
                        Units.feetToMeters(20.0),
                        Rotation2d.fromDegrees(-180))),
                true),
            robot.drive,
            Calibration.Auto.TRACKER_CONSTANTS));

    addState("Shoot 3 Balls", new ShootSequence(robot));

    addState(
        "Drive Past Auto Line",
        new SparkTrajectoryTracker(
            trajectoryCreator.getTrajectory(
                List.of(
                    new Pose2d(
                        Units.feetToMeters(15.3),
                        Units.feetToMeters(20.0),
                        Rotation2d.fromDegrees(-180)),
                    new Pose2d(
                        Units.feetToMeters(18.0),
                        Units.feetToMeters(24.5),
                        Rotation2d.fromDegrees(-180))),
                true),
            robot.drive,
            Calibration.Auto.TRACKER_CONSTANTS));

    addState(
        "Path and Trench Intake",
        new ParallelRaceCoordinator(
            "Trench Intake",
            new IntakeMacro(
                robot.intake, robot.intakeDeploy, robot.revolver, robot.tower, robot.antiJamRoller),
            new SparkTrajectoryTracker(
                trajectoryCreator.getTrajectory(
                    List.of(
                        new Pose2d(
                            Units.feetToMeters(18.0),
                            Units.feetToMeters(24.5),
                            Rotation2d.fromDegrees(-180)),
                        new Pose2d(
                            Units.feetToMeters(27.0),
                            Units.feetToMeters(24.5),
                            Rotation2d.fromDegrees(-180))),
                    true),
                robot.drive,
                Calibration.Auto.TRACKER_CONSTANTS)));

    addState("Shoot 3 Balls", new ShootSequence(robot));
  }
}
