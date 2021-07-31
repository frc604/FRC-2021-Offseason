package com._604robotics.robot2020.auto.macros;

import com._604robotics.robot2020.constants.Calibration;
import com._604robotics.robot2020.modules.Drive;
import com._604robotics.robot2020.modules.Limelight;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.prefabs.controller.AutoTargetPIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

public class AutoCenterMacro extends Coordinator {
  private Drive.ArcadeDrive arcade;
  private Limelight limelight;

  private AutoTargetPIDController anglePID;

  public AutoCenterMacro(Drive.ArcadeDrive drive, Limelight limelight, boolean innerGoal) {
    this.arcade = drive;
    this.limelight = limelight;

    anglePID =
        new AutoTargetPIDController(
            Calibration.AutoAlign.kP,
            0.0,
            Calibration.AutoAlign.kD,
            new TrapezoidProfile.Constraints(50, 10),
            new SimpleMotorFeedforward(0.0, 0.025),
            () -> -1.0 * limelight.limelightX.get(),
            this::setOutput);

    if (innerGoal) {
      anglePID.setTolerance(Calibration.AutoAlign.ABSOLUTE_TOLERANCE_INNER_GOAL);
    } else {
      anglePID.setTolerance(Calibration.AutoAlign.ABSOLUTE_TOLERANCE_OUTER_GOAL);
    }
  }

  public void setOutput(double output) {
    arcade.rotatePower.set(output);
  }

  @Override
  protected void begin() {
    anglePID.setInitialSetpoint(-limelight.limelightX.get());
    SmartDashboard.putNumber("TRAP", -limelight.limelightX.get());
  }

  @Override
  public boolean run() {
    limelight.scan.activate();

    SmartDashboard.putNumber("PID ERRROR", anglePID.getPositionError());

    if (limelight.limelightHasTargets.get()) {
      anglePID.setEnabled(true);
      System.out.println(anglePID.get());
    }

    return true;
  }

  public boolean aligned() {
    return anglePID.atGoal();
  }

  @Override
  public void end() {
    anglePID.setEnabled(false);
    anglePID.reset();
  }
}
