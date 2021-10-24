package com._604robotics.robot2020.auto.macros;

import com._604robotics.robot2020.Robot2020;
import com._604robotics.robotnik.prefabs.coordinators.ParallelRaceCoordinator;

public class ShootSequence extends ParallelRaceCoordinator {
  public ShootSequence(Robot2020 robot) {
    super(ShootSequence.class);

    addCoordinators(new ShooterControlMacro(robot.shooter), new CombinedFeedMacro(robot));
  }
}
