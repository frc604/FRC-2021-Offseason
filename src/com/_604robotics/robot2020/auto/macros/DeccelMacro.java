package com._604robotics.robot2020.auto.macros;

import com._604robotics.robot2020.modules.Drive;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.prefabs.motion.MotionConstraints;
import com._604robotics.robotnik.prefabs.motion.MotionState;
import com._604robotics.robotnik.prefabs.motion.TrapezoidalMotionProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;

public class DeccelMacro extends Coordinator {
  private Drive drivetrain;
  private Drive.SparkDrivePID sparkPID;
  private final Timer timer = new Timer();
  private TrapezoidalMotionProfile leftProfile;
  private TrapezoidalMotionProfile rightProfile;

  public DeccelMacro(Drive drivetrain) {
    this.drivetrain = drivetrain;
    sparkPID = drivetrain.new SparkDrivePID();

    DifferentialDriveWheelSpeeds initialSpeeds = drivetrain.getWheelSpeeds();
    leftProfile =
        new TrapezoidalMotionProfile(
            new MotionState(initialSpeeds.leftMetersPerSecond, 0),
            new MotionState(),
            new MotionConstraints(1, 1));
    rightProfile =
        new TrapezoidalMotionProfile(
            new MotionState(initialSpeeds.rightMetersPerSecond, 0),
            new MotionState(),
            new MotionConstraints(1, 1));
  }

  @Override
  protected void begin() {
    DifferentialDriveWheelSpeeds initialSpeeds = drivetrain.getWheelSpeeds();
    leftProfile.setParams(
        new MotionState(initialSpeeds.leftMetersPerSecond, 0),
        new MotionState(),
        new MotionConstraints(1, 5));
    rightProfile.setParams(
        new MotionState(initialSpeeds.rightMetersPerSecond, 0),
        new MotionState(),
        new MotionConstraints(1, 5));
    timer.reset();
    timer.start();
  }

  @Override
  public boolean run() {
    sparkPID.leftVel.set(leftProfile.sample(timer.get()).position);
    sparkPID.rightVel.set(rightProfile.sample(timer.get()).position);

    sparkPID.leftFF.set(0.0);
    sparkPID.rightFF.set(0.0);

    sparkPID.activate();
    return (timer.get() <= leftProfile.getTotalTime()
        && timer.get() <= rightProfile.getTotalTime());
  }

  @Override
  public void end() {
    sparkPID.leftVel.set(0.0);
    sparkPID.rightVel.set(0.0);

    sparkPID.leftFF.set(0.0);
    sparkPID.rightFF.set(0.0);

    sparkPID.activate();
  }
}
