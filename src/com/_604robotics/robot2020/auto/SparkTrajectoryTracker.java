package com._604robotics.robot2020.auto;

import com._604robotics.robot2020.modules.Drive;
import com._604robotics.robotnik.prefabs.auto.TrackerConstants;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class SparkTrajectoryTracker extends TrackingCoordinator {
  private Drive.SparkDrivePID sparkPID;

  public SparkTrajectoryTracker(
      Trajectory trajectory, Drive drivetrain, TrackerConstants constants) {
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
