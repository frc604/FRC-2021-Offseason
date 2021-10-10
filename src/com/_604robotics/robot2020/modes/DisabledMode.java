package com._604robotics.robot2020.modes;

import com._604robotics.quixsam.mathematics.Interpolatable;
import com._604robotics.robot2020.Robot2020;
import com._604robotics.robot2020.constants.Calibration;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;
import com._604robotics.robotnik.prefabs.auto.QuikPlanLive;
import com._604robotics.robotnik.prefabs.vision.VisionCamera.GamePiece;
import com._604robotics.robotnik.prefabs.vision.VisionManager;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.Pair;
import java.util.List;

public class DisabledMode extends Coordinator {
  private static final Logger logger = new Logger(DisabledMode.class);

  private final com._604robotics.robot2020.Robot2020 robot;

  public QuikPlanLive livePlan;

  private boolean sent = false;

  public DisabledMode(Robot2020 robot) {
    this.robot = robot;
    this.livePlan = new QuikPlanLive();
  }

  @Override
  public void begin() {
    Calibration.Auto.XDISTACEBALLCAM.set(
        new Pair<Double, Interpolatable<Double>>(9.543333, Interpolatable.interDouble(6.096)),
        new Pair<Double, Interpolatable<Double>>(9.036667, Interpolatable.interDouble(5.7912)),
        new Pair<Double, Interpolatable<Double>>(8.54, Interpolatable.interDouble(5.4864)),
        new Pair<Double, Interpolatable<Double>>(7.96, Interpolatable.interDouble(5.1816)),
        new Pair<Double, Interpolatable<Double>>(7.27, Interpolatable.interDouble(4.8768)),
        new Pair<Double, Interpolatable<Double>>(5.75, Interpolatable.interDouble(4.572)),
        new Pair<Double, Interpolatable<Double>>(5.1, Interpolatable.interDouble(4.2672)),
        new Pair<Double, Interpolatable<Double>>(4.06667, Interpolatable.interDouble(3.9624)),
        new Pair<Double, Interpolatable<Double>>(3.05667, Interpolatable.interDouble(3.6576)),
        new Pair<Double, Interpolatable<Double>>(1.88667, Interpolatable.interDouble(3.3528)),
        new Pair<Double, Interpolatable<Double>>(0.51, Interpolatable.interDouble(3.048)),
        new Pair<Double, Interpolatable<Double>>(-1.68, Interpolatable.interDouble(2.7432)),
        new Pair<Double, Interpolatable<Double>>(-3.7733, Interpolatable.interDouble(2.4384)),
        new Pair<Double, Interpolatable<Double>>(-6.4367, Interpolatable.interDouble(2.1336)),
        new Pair<Double, Interpolatable<Double>>(-10.877, Interpolatable.interDouble(1.8288)),
        new Pair<Double, Interpolatable<Double>>(-15.513, Interpolatable.interDouble(1.524)));

    SmartDashboard.putBoolean("Enable Searching", false);
    SmartDashboard.putBoolean("Send Obstacles", false);
    sent = false;

    SmartDashboard.putBoolean("Load Selected Path", false);
  }

  @Override
  public boolean run() {
    // robot.drive.updateOdometry();

    // // Search Gamepieces
    // if (SmartDashboard.getBoolean("Enable Searching", false)) {
    //   VisionManager.getInstance().update();
    //   List<Translation2d> balls =
    //       robot.ballCam.searchGamePiece(robot.drive.getPose(), GamePiece.POWERCELL);
    //   String ballString = "";
    //   for (Translation2d ball : balls) {
    //     ballString = ballString + ball.toString() + " ";
    //   }
    //   SmartDashboard.putString("Balls", ballString);

    //   if (SmartDashboard.getBoolean("Send Obstacles", false) && !sent) {
    //     livePlan.publishObstacles(balls, 0.088);
    //     sent = true;
    //   }
    // }

    // if (SmartDashboard.getBoolean("Load Selected Path", false)) {
    //   robot.autonomousMode.getReader().loadChosenFile();
    //   SmartDashboard.putBoolean("Load Selected Path", false);
    // }

    return true;
  }

  @Override
  public void end() {}
}
