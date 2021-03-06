package com._604robotics.robot2020.auto.paths;

import com._604robotics.robot2020.Robot2020;
import com._604robotics.robot2020.auto.QuixPlanSwerveTrajectoryTracker;
import com._604robotics.robot2020.auto.macros.AutoAimMacro;
import com._604robotics.robot2020.auto.macros.CombinedFeedMacro;
import com._604robotics.robot2020.auto.macros.FeedMacro;
import com._604robotics.robot2020.auto.macros.FeedOTFMacro;
import com._604robotics.robot2020.auto.macros.IntakeMacro;
import com._604robotics.robot2020.auto.macros.ShooterControlMacro;
import com._604robotics.robot2020.constants.Calibration;
import com._604robotics.robotnik.prefabs.auto.QuikPlanReader;
import com._604robotics.robotnik.prefabs.auto.QuikPlanSwerveReader;
import com._604robotics.robotnik.prefabs.coordinators.ParallelCoordinator;
import com._604robotics.robotnik.prefabs.coordinators.ParallelRaceCoordinator;
import com._604robotics.robotnik.prefabs.coordinators.StatefulCoordinator;

public class QuikPlanPreGen extends StatefulCoordinator {
  public QuikPlanPreGen(Robot2020 robot, QuikPlanSwerveReader reader) {
    super(QuikPlanPreGen.class);

    QuixPlanSwerveTrajectoryTracker tracker = new QuixPlanSwerveTrajectoryTracker(
      reader, robot.drive, Calibration.Auto.TRACKER_CONSTANTS);

    addState(
      "SOTF",
      new ParallelRaceCoordinator(
          "SOTF1",
          tracker,
          new ShooterControlMacro(robot.shooter),
          new FeedOTFMacro(robot.intake, robot.intakeDeploy, robot.revolver, robot.tower, robot.antiJamRoller, () -> tracker.doShoot())
      )
    );
    addState("AUTOAIM", new AutoAimMacro(robot.drive, robot.limelight).withTimeout(0.5));
    addState("FINISH", new CombinedFeedMacro(robot).withTimeout(5));
 }
}