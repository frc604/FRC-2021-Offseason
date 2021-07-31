package com._604robotics.robot2020.auto;

import com._604robotics.robot2020.modules.Drive;
import com._604robotics.robotnik.prefabs.auto.QuikPlanReader;
import com._604robotics.robotnik.prefabs.auto.TrackerConstants;

public class QuixPlanTrajectoryTracker extends QuikPlanTrackingCoordinator {
  private Drive.DrivePID PID;

  public QuixPlanTrajectoryTracker(
      QuikPlanReader reader, Drive drivetrain, TrackerConstants constants) {
    super(reader, drivetrain, constants);

    PID = drivetrain.new DrivePID();
  }

  @Override
  public void useOutput(
      double leftSpeedSetpoint,
      double rightSpeedSetpoint,
      double leftFeedforward,
      double rightFeedforward) {
    PID.leftVel.set(leftSpeedSetpoint);
    PID.rightVel.set(rightSpeedSetpoint);

    PID.leftFF.set(leftFeedforward);
    PID.rightFF.set(rightFeedforward);

    PID.activate();
  }

  @Override
  public void stop() {
    PID.leftVel.set(0.0);
    PID.rightVel.set(0.0);

    PID.leftFF.set(0.0);
    PID.rightFF.set(0.0);

    PID.activate();
    PID.end();
  }
}
