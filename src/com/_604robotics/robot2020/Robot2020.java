package com._604robotics.robot2020;

import com._604robotics.robot2020.constants.Ports;
import com._604robotics.robot2020.modes.*;
import com._604robotics.robot2020.modules.*;
import com._604robotics.robotnik.DashboardManager;
import com._604robotics.robotnik.Robot;
import com._604robotics.robotnik.prefabs.modules.Shifter;
import com._604robotics.robotnik.prefabs.motorcontrol.PowerMonitor;
import com._604robotics.robotnik.prefabs.vision.PhotonVisionCamera;
import com._604robotics.robotnik.prefabs.vision.VisionManager;
import edu.wpi.first.wpilibj.RobotBase;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class Robot2020 extends Robot {

  public static void main(String[] args) {
    RobotBase.startRobot(Robot2020::new);
  }

  private Robot2020() {
    DashboardManager.getInstance().registerRobot(this);
  }

  public final PowerMonitor powerMonitor =
      PowerMonitor.getInstance(Ports.PDP_MODULE, Ports.COMPRESSOR);

  public final Drive drive = addModule(new Drive());

  public final DisabledMode disabledMode = setDisabledMode(new DisabledMode(this));
  public final TeleopMode teleopMode = setTeleopMode(new TeleopMode(this));
  public final AutonomousMode autonomousMode = setAutonomousMode(new AutonomousMode(this));
}
