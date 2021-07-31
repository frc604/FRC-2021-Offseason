package com._604robotics.robot2020.auto;

import com._604robotics.robot2020.modules.Drive;
import com._604robotics.robotnik.prefabs.auto.TrackerConstants;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;

public class TrajectoryTracker extends TrackingCoordinator {
  private Drive.TankDriveVolts tank;
  private final PIDController leftVelController;
  private final PIDController rightVelController;
  private Drive drivetrain;

  public TrajectoryTracker(Trajectory trajectory, Drive drivetrain, TrackerConstants constants) {
    super(trajectory, drivetrain, constants);

    this.drivetrain = drivetrain;

    tank = drivetrain.new TankDriveVolts();

    leftVelController = new PIDController(constants.kP, 0, 0);
    rightVelController = new PIDController(constants.kP, 0, 0);
  }

  @Override
  public void useOutput(
      double leftSpeedSetpoint,
      double rightSpeedSetpoint,
      double leftFeedforward,
      double rightFeedforward) {
    double leftOutput;
    double rightOutput;

    leftOutput =
        leftFeedforward
            + leftVelController.calculate(
                drivetrain.getWheelSpeeds().leftMetersPerSecond, leftSpeedSetpoint);

    rightOutput =
        rightFeedforward
            + rightVelController.calculate(
                drivetrain.getWheelSpeeds().rightMetersPerSecond, rightSpeedSetpoint);

    tank.leftVolts.set(leftOutput);
    tank.rightVolts.set(rightOutput);

    tank.activate();
  }

  @Override
  public void stop() {
    tank.leftVolts.set(0.0);
    tank.rightVolts.set(0.0);

    tank.activate();
  }
}
