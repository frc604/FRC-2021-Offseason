package com._604robotics.robot2020.auto;

import com._604robotics.robot2020.constants.Calibration;
import com._604robotics.robot2020.modules.Drive;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.prefabs.auto.FalconDashboard;
import com._604robotics.robotnik.prefabs.auto.QuikPlanReader;
import com._604robotics.robotnik.prefabs.auto.TrackerConstants;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.LinearPlantInversionFeedforward;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.system.LinearSystem;
import edu.wpi.first.wpilibj.system.plant.LinearSystemId;
import edu.wpi.first.wpiutil.math.numbers.N2;
import java.util.List;

public class QuikPlanTrackingCoordinator extends Coordinator {
  private final Timer timer = new Timer();
  private QuikPlanReader reader;
  private Drive drivetrain;
  private DifferentialDriveWheelSpeeds prevSpeeds;
  private final RamseteController controller;
  private final SimpleMotorFeedforward feedforward;
  private final LinearSystem<N2, N2, N2> drivetrainSystem;
  private final LinearPlantInversionFeedforward<N2, N2, N2> feedforwardButBetter;
  private double prevTime;
  private double totalTime;
  private Field2d field = new Field2d();

  public QuikPlanTrackingCoordinator(
      QuikPlanReader reader, Drive drivetrain, TrackerConstants constants) {
    this.reader = reader;
    this.drivetrain = drivetrain;
    SmartDashboard.putString(String.valueOf(this.hashCode()), String.valueOf(this.hashCode()));
    // drivetrainSystem = LinearSystemId.createDrivetrainVelocitySystem(DCMotor.getNEO(4), 50,
    // Calibration.Drive.WHEEL_DIAMETER / 2, Calibration.Drive.TRACK_WIDTH, 6.237, 9.5625);
    drivetrainSystem = LinearSystemId.identifyDrivetrainSystem(2.47, 0.525, 2.648, 0.562);
    feedforwardButBetter = new LinearPlantInversionFeedforward<>(drivetrainSystem, 0.02);

    SmartDashboard.putNumber("PID P", Calibration.Auto.KP_DRIVE_VELCOTIY);

    controller = new RamseteController(constants.b, constants.zeta);
    feedforward = constants.feedforward;
  }

  @Override
  public void begin() {
    prevTime = 0;
    SmartDashboard.putData("field", field);

    List<Double> initialState = reader.getState(0.0);
    drivetrain.zeroOdometry(
        new Pose2d(initialState.get(0), initialState.get(1), new Rotation2d(initialState.get(2))));

    FalconDashboard.getInstance().publishRobotPose(drivetrain.getPose());
    FalconDashboard.getInstance()
        .publishPathPose(
            new Pose2d(
                initialState.get(0), initialState.get(1), new Rotation2d(initialState.get(2))));

    prevSpeeds =
        drivetrain.driveKinematics.toWheelSpeeds(
            new ChassisSpeeds(initialState.get(3), 0, initialState.get(4)));
    totalTime = reader.getTotalTime();
    timer.reset();
    timer.start();
  }

  @Override
  public boolean run() {
    // System.out.println(drivetrain.getPose());
    double curTime = timer.get();
    double dt = curTime - prevTime;

    if (timer.hasElapsed(totalTime)) {
      return false;
    } else {
      List<Double> state = reader.getState(curTime);
      var targetWheelSpeeds =
          drivetrain.driveKinematics.toWheelSpeeds(
              controller.calculate(
                  drivetrain.getPose(),
                  new Pose2d(state.get(0), state.get(1), new Rotation2d(state.get(2))),
                  state.get(3),
                  state.get(4)));
      var leftSpeedSetpoint = targetWheelSpeeds.leftMetersPerSecond;
      var rightSpeedSetpoint = targetWheelSpeeds.rightMetersPerSecond;

      double leftFeedforward =
          feedforward.calculate(
              leftSpeedSetpoint, (leftSpeedSetpoint - prevSpeeds.leftMetersPerSecond) / dt);

      double rightFeedforward =
          feedforward.calculate(
              rightSpeedSetpoint, (rightSpeedSetpoint - prevSpeeds.rightMetersPerSecond) / dt);

      // List<Double> futureState = reader.getState(curTime + 0.02);
      // var betterFeedforward = feedforwardButBetter.calculate(VecBuilder.fill(state.get(5),
      // state.get(6)), VecBuilder.fill(futureState.get(5), futureState.get(6)));

      useOutput(leftSpeedSetpoint, rightSpeedSetpoint, leftFeedforward, rightFeedforward);

      prevTime = curTime;
      prevSpeeds = targetWheelSpeeds;

      // field.setRobotPose(drivetrain.getPose());

      FalconDashboard.getInstance().publishRobotPose(drivetrain.getPose());
      FalconDashboard.getInstance()
          .publishPathPose(new Pose2d(state.get(0), state.get(1), new Rotation2d(state.get(2))));
      return true;
    }
  }

  @Override
  public void end() {
    drivetrain.setIdleMode(IdleMode.kBrake);
    stop();
  }

  public void useOutput(
      double leftSpeedSetpoint,
      double rightSpeedSetpoint,
      double leftFeedforward,
      double rightFeedforward) {}

  public void stop() {}
}
