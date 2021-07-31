package com._604robotics.robot2020.auto.paths;

import com._604robotics.robot2020.Robot2020;
import com._604robotics.robot2020.auto.QuixPlanLiveSparkTrajectoryTracker;
import com._604robotics.robot2020.auto.macros.IntakeMacro;
import com._604robotics.robot2020.auto.macros.ShootSequence;
import com._604robotics.robot2020.auto.macros.ShooterControlMacro;
import com._604robotics.robot2020.constants.Calibration;
import com._604robotics.robotnik.prefabs.auto.QuikPlanLive;
import com._604robotics.robotnik.prefabs.coordinators.ParallelRaceCoordinator;
import com._604robotics.robotnik.prefabs.coordinators.SleepCoordinator;
import com._604robotics.robotnik.prefabs.coordinators.StatefulCoordinator;

public class QuikPlanSearch extends StatefulCoordinator {
  public QuikPlanSearch(Robot2020 robot, QuikPlanLive reader) {
    super(Slalom.class);

    addState(
        "Path and Intake",
        new ParallelRaceCoordinator(
            "pp",
            new ParallelRaceCoordinator(
                "Search Intake",
                new IntakeMacro(
                    robot.intake,
                    robot.intakeDeploy,
                    robot.revolver,
                    robot.tower,
                    robot.antiJamRoller),
                new QuixPlanLiveSparkTrajectoryTracker(
                    reader, robot.drive, Calibration.Auto.TRACKER_CONSTANTS)),
            new StatefulCoordinator("IDK")
                .addState("Timer", new SleepCoordinator(reader.getTotalTime() - 3))
                .addState("Shoot", new ShooterControlMacro(robot.shooter))));
    addState("Shoot 3 Balls", new ShootSequence(robot));
  }
}
