package com._604robotics.robot2020.auto.macros;

import com._604robotics.robot2020.Robot2020;
import com._604robotics.robotnik.prefabs.coordinators.ParallelCoordinator;
import com._604robotics.robotnik.prefabs.coordinators.StatefulCoordinator;

public class CombinedFeedMacro extends ParallelCoordinator {

  public CombinedFeedMacro(Robot2020 robot) {
    super(CombinedFeedMacro.class);

    addCoordinators(
        new FeedMacro(robot.revolver, robot.tower, robot.antiJamRoller),
        new ShooterControlMacro(robot.shooter)
    );
  }
}
