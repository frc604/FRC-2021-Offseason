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

public class Full10ball extends StatefulCoordinator {
  public Full10ball(Robot2020 robot, TrajectoryCreator trajectoryCreator) {
    super(Full10ball.class);

    addState(
        "Reverse into Trench",
        new SparkTrajectoryTracker(
            trajectoryCreator.getTrajectory(
                List.of(
                    new Pose2d(
                        Units.feetToMeters(2.5),
                        Units.feetToMeters(2.5),
                        Rotation2d.fromDegrees(0)),
                    new Pose2d(
                        Units.feetToMeters(10),
                        Units.feetToMeters(7.5),
                        Rotation2d.fromDegrees(90))),
                false),
            robot.drive,
            Calibration.Auto.TRACKER_CONSTANTS));
    // addState(
    //     "Traverse to other side",
    //     new SparkTrajectoryTracker(
    //         trajectoryCreator.getTrajectory(
    //             List.of(
    //                 new Pose2d(
    //                     Units.feetToMeters(20.0),
    //                     Units.feetToMeters(2.2),
    //                     Rotation2d.fromDegrees(-180)),
    //                 new Pose2d(
    //                     Units.feetToMeters(16.0),
    //                     Units.feetToMeters(14.0),
    //                     Rotation2d.fromDegrees(85)),
    //                 new Pose2d(
    //                     Units.feetToMeters(12.0),
    //                     Units.feetToMeters(19.0),
    //                     Rotation2d.fromDegrees(-180))),
    //             false),
    //         robot.drive,
    //         Calibration.Auto.TRACKER_CONSTANTS));
    // addState(
    //     "Reverse into trusses",
    //     new SparkTrajectoryTracker(
    //         trajectoryCreator.getTrajectory(
    //             List.of(
    //                 new Pose2d(
    //                     Units.feetToMeters(12.0),
    //                     Units.feetToMeters(19.0),
    //                     Rotation2d.fromDegrees(-180)),
    //                 new Pose2d(
    //                     Units.feetToMeters(20.4),
    //                     Units.feetToMeters(18.5),
    //                     Rotation2d.fromDegrees(110))),
    //             true),
    //         robot.drive,
    //         Calibration.Auto.TRACKER_CONSTANTS));
    // addState(
    //     "Align to Trench",
    //     new SparkTrajectoryTracker(
    //         trajectoryCreator.getTrajectory(
    //             List.of(
    //                 new Pose2d(
    //                     Units.feetToMeters(20.4),
    //                     Units.feetToMeters(18.5),
    //                     Rotation2d.fromDegrees(110)),
    //                 new Pose2d(
    //                     Units.feetToMeters(15.5),
    //                     Units.feetToMeters(24.75),
    //                     Rotation2d.fromDegrees(-180))),
    //             false),
    //         robot.drive,
    //         Calibration.Auto.TRACKER_CONSTANTS));
    // addState(
    //     "Drive into Trench",
    //     new SparkTrajectoryTracker(
    //         trajectoryCreator.getTrajectory(
    //             List.of(
    //                 new Pose2d(
    //                     Units.feetToMeters(15.5),
    //                     Units.feetToMeters(24.75),
    //                     Rotation2d.fromDegrees(-180)),
    //                 new Pose2d(
    //                     Units.feetToMeters(26.5),
    //                     Units.feetToMeters(24.75),
    //                     Rotation2d.fromDegrees(-180))),
    //             true),
    //         robot.drive,
    //         Calibration.Auto.TRACKER_CONSTANTS));
  }
}
