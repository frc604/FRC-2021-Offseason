package com._604robotics.robot2020.auto.angular;

import com._604robotics.robot2020.modules.Drive;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.prefabs.auto.TrackerConstants;
import com._604robotics.robotnik.prefabs.auto.angular.AngularTrajectoryState;
import com._604robotics.robotnik.prefabs.auto.angular.TurnInPlaceTrajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AngularTrackingCoordinator extends Coordinator {
  private final Timer timer = new Timer();
  private TurnInPlaceTrajectory trajectory;
  private Drive drivetrain;
  private DifferentialDriveWheelSpeeds prevSpeeds;
  private final RamseteController controller;
  private final SimpleMotorFeedforward feedforward;
  private double totalTime;
  private double prevTime;

  public AngularTrackingCoordinator(
      TurnInPlaceTrajectory trajectory, Drive drivetrain, TrackerConstants constants) {
    this.trajectory = trajectory;
    this.drivetrain = drivetrain;

    controller = new RamseteController(constants.b, constants.zeta);
    feedforward = constants.feedforward;

    totalTime = trajectory.getTotalTimeSeconds();
  }

  @Override
  public void begin() {
    prevTime = 0;

    drivetrain.zeroOdometry(trajectory.getInitialPose());

    var initialState = trajectory.sample(0, 0);
    prevSpeeds =
        drivetrain.driveKinematics.toWheelSpeeds(
            new ChassisSpeeds(0, 0, initialState.angularVelocity));
    timer.reset();
    timer.start();
  }

  @Override
  public boolean run() {
    System.out.println(drivetrain.getPose());
    double curTime = timer.get();
    double dt = curTime - prevTime;

    AngularTrajectoryState currentState = trajectory.sample(curTime, dt);

    var targetWheelSpeeds =
        drivetrain.driveKinematics.toWheelSpeeds(
            controller.calculate(
                drivetrain.getPose(), currentState.pose, 0.0, currentState.angularVelocity));

    var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
    var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

    double leftFeedforward =
        feedforward.calculate(
            leftSpeedSetpoint, (leftSpeedSetpoint - prevSpeeds.leftMetersPerSecond) / dt);

    double rightFeedforward =
        feedforward.calculate(
            rightSpeedSetpoint, (rightSpeedSetpoint - prevSpeeds.rightMetersPerSecond) / dt);

    SmartDashboard.putNumber("LEFT FF", leftFeedforward);
    SmartDashboard.putNumber("RIGHT FF", rightFeedforward);

    SmartDashboard.putNumber("POSE", currentState.pose.getRotation().getDegrees());

    System.out.println("Left" + leftSpeedSetpoint);
    System.out.println("LeftFF" + leftFeedforward);
    System.out.println("RightFF" + rightFeedforward);
    System.out.println("Right" + rightSpeedSetpoint);

    useOutput(leftSpeedSetpoint, rightSpeedSetpoint, leftFeedforward, rightFeedforward);

    prevTime = curTime;
    prevSpeeds = targetWheelSpeeds;

    return !timer.hasPeriodPassed(totalTime);
  }

  @Override
  public void end() {
    stop();
    timer.stop();
  }

  public void useOutput(
      double leftSpeedSetpoint,
      double rightSpeedSetpoint,
      double leftFeedforward,
      double rightFeedforward) {}

  public void stop() {}
}
