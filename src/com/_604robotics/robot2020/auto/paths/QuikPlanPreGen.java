package com._604robotics.robot2020.auto.paths;

import com._604robotics.robot2020.Robot2020;
import com._604robotics.robot2020.auto.QuixPlanSparkTrajectoryTracker;
import com._604robotics.robot2020.auto.macros.DeccelMacro;
import com._604robotics.robot2020.constants.Calibration;
import com._604robotics.robotnik.prefabs.auto.QuikPlanReader;
import com._604robotics.robotnik.prefabs.coordinators.StatefulCoordinator;

public class QuikPlanPreGen extends StatefulCoordinator {
  public QuikPlanPreGen(Robot2020 robot, QuikPlanReader reader) {
    super(Slalom.class);

    addState(
        "Driver Slalom",
        new QuixPlanSparkTrajectoryTracker(
            reader, robot.drive, Calibration.Auto.TRACKER_CONSTANTS));
    addState("Deccel", new DeccelMacro(robot.drive));
  }
}
