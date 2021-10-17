package com._604robotics.robot2020.auto.paths;

import com._604robotics.robot2020.Robot2020;
import com._604robotics.robot2020.auto.QuixPlanSwerveTrajectoryTracker;
import com._604robotics.robot2020.constants.Calibration;
import com._604robotics.robotnik.prefabs.auto.QuikPlanReader;
import com._604robotics.robotnik.prefabs.auto.QuikPlanSwerveReader;
import com._604robotics.robotnik.prefabs.coordinators.StatefulCoordinator;

public class QuikPlanPreGen extends StatefulCoordinator {
  public QuikPlanPreGen(Robot2020 robot, QuikPlanSwerveReader reader) {
    super(QuikPlanPreGen.class);

    addState(
        "Driver Slalom",
        new QuixPlanSwerveTrajectoryTracker(
            reader, robot.drive, Calibration.Auto.TRACKER_CONSTANTS));
 }
}