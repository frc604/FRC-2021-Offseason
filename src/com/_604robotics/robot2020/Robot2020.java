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
    VisionManager.getInstance().registerCamera(ballCam, "mmal_service_16.1");
  }

  public final PowerMonitor powerMonitor =
      PowerMonitor.getInstance(Ports.PDP_MODULE, Ports.COMPRESSOR);

  public final Drive drive = addModule(new Drive());

  public final Shifter shifter = addModule(drive.shifter);

  public final Intake intake = addModule(new Intake());
  public final IntakeDeploy intakeDeploy = addModule(new IntakeDeploy());

  public final Revolver revolver = addModule(new Revolver());

  public final AntiJamRoller antiJamRoller = addModule(new AntiJamRoller());

  public final Shooter shooter = addModule(new Shooter());

  public final Tower tower = addModule(new Tower());

  public final Limelight limelight = addModule(new Limelight());

  public final PhotonVisionCamera ballCam =
      addModule(
          new PhotonVisionCamera(
              "mmal_service_16.1", new Vector3D(-0.10813321, 0.01653152, 1.03617424), -20));

  public final Climber climber = addModule(new Climber());

  public final DisabledMode disabledMode = setDisabledMode(new DisabledMode(this));
  public final TeleopMode teleopMode = setTeleopMode(new TeleopMode(this));
  public final AutonomousMode autonomousMode = setAutonomousMode(new AutonomousMode(this));
}
