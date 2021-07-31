package com._604robotics.robot2020.auto.macros;

import com._604robotics.robot2020.Robot2020;
import com._604robotics.robotnik.prefabs.coordinators.StatefulCoordinator;

public class CombinedFeedMacro extends StatefulCoordinator {

  private AutoCenterMacro autoCenterMacro;

  public CombinedFeedMacro(Robot2020 robot) {
    super(CombinedFeedMacro.class);

    // this.autoCenterMacro =
    //     new AutoCenterMacro(robot.drive.new ArcadeDrive(), robot.limelight, false);

    // addState("Auto Center to Outer Goal",
    // autoCenterMacro.withInterrupt(autoCenterMacro::aligned));
    // addState("Short delay", new SleepCoordinator(3));
    addState(
        "Fire", new FeedMacro(robot.revolver, robot.tower, robot.antiJamRoller).withTimeout(5));
  }
}
