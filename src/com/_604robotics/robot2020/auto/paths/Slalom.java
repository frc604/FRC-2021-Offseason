package com._604robotics.robot2020.auto.paths;

import com._604robotics.robot2020.Robot2020;
import com._604robotics.robot2020.auto.SparkTrajectoryTracker;
import com._604robotics.robot2020.constants.Calibration;
import com._604robotics.robotnik.prefabs.auto.TrajectoryCreator;
import com._604robotics.robotnik.prefabs.coordinators.StatefulCoordinator;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Units;
import java.util.List;

public class Slalom extends StatefulCoordinator {
  public Slalom(Robot2020 robot, TrajectoryCreator trajectoryCreator) {
    super(Slalom.class);

    addState(
        "Driver Slalom",
        new SparkTrajectoryTracker(
            trajectoryCreator.getTrajectory(
                List.of(
                    new Pose2d(
                        Units.feetToMeters(2.5),
                        Units.feetToMeters(2.5),
                        Rotation2d.fromDegrees(0)),
                    new Pose2d(
                        Units.feetToMeters(7.5),
                        Units.feetToMeters(5),
                        Rotation2d.fromDegrees(75.495)),
                    new Pose2d(
                        Units.feetToMeters(15), Units.feetToMeters(8.5), Rotation2d.fromDegrees(0)),
                    new Pose2d(
                        Units.feetToMeters(22.5),
                        Units.feetToMeters(5),
                        Rotation2d.fromDegrees(-78.293)),
                    new Pose2d(
                        Units.feetToMeters(25), Units.feetToMeters(1.5), Rotation2d.fromDegrees(0)),
                    new Pose2d(
                        Units.feetToMeters(28.5),
                        Units.feetToMeters(5),
                        Rotation2d.fromDegrees(90.896)),
                    new Pose2d(
                        Units.feetToMeters(25),
                        Units.feetToMeters(8.5),
                        Rotation2d.fromDegrees(-179.651)),
                    new Pose2d(
                        Units.feetToMeters(22.465),
                        Units.feetToMeters(5.072),
                        Rotation2d.fromDegrees(-102.045)),
                    new Pose2d(
                        Units.feetToMeters(15),
                        Units.feetToMeters(1.5),
                        Rotation2d.fromDegrees(180)),
                    new Pose2d(
                        Units.feetToMeters(7.5),
                        Units.feetToMeters(5),
                        Rotation2d.fromDegrees(107.788)),
                    new Pose2d(
                        Units.feetToMeters(2.5),
                        Units.feetToMeters(7.5),
                        Rotation2d.fromDegrees(180))),
                false),
            robot.drive,
            Calibration.Auto.TRACKER_CONSTANTS));
  }
}
