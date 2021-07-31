package com._604robotics.robot2020.auto.angular;

import com._604robotics.robot2020.modules.Drive;
import com._604robotics.robotnik.prefabs.auto.TrackerConstants;
import com._604robotics.robotnik.prefabs.auto.angular.TurnInPlaceTrajectory;

public class SparkAngularTrajectoryTracker extends AngularTrackingCoordinator {
  private Drive.SparkDrivePID sparkPID;

  public SparkAngularTrajectoryTracker(
      TurnInPlaceTrajectory trajectory, Drive drivetrain, TrackerConstants constants) {
    super(trajectory, drivetrain, constants);

    sparkPID = drivetrain.new SparkDrivePID();
  }

  @Override
  public void useOutput(
      double leftSpeedSetpoint,
      double rightSpeedSetpoint,
      double leftFeedforward,
      double rightFeedforward) {
    sparkPID.leftVel.set(leftSpeedSetpoint);
    sparkPID.rightVel.set(rightSpeedSetpoint);

    sparkPID.leftFF.set(leftFeedforward);
    sparkPID.rightFF.set(rightFeedforward);

    sparkPID.activate();
  }

  @Override
  public void stop() {
    sparkPID.leftVel.set(0.0);
    sparkPID.rightVel.set(0.0);

    sparkPID.leftFF.set(0.0);
    sparkPID.rightFF.set(0.0);

    sparkPID.activate();
  }
}
