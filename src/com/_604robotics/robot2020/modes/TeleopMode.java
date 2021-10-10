package com._604robotics.robot2020.modes;

import com._604robotics.marionette.InputPlayer;
import com._604robotics.marionette.InputRecorder;
import com._604robotics.marionette.InputRecording;
import com._604robotics.marionette.MarionetteJoystick;
import com._604robotics.robot2020.constants.Calibration;
import com._604robotics.robot2020.modules.AntiJamRoller;
import com._604robotics.robot2020.modules.Intake;
import com._604robotics.robot2020.modules.IntakeDeploy;
import com._604robotics.robot2020.modules.Revolver;
import com._604robotics.robot2020.modules.Swerve;
import com._604robotics.robot2020.modules.Tower;
import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;
import com._604robotics.robotnik.prefabs.flow.Toggle;
import com._604robotics.robotnik.prefabs.inputcontroller.xbox.XboxController;

public class TeleopMode extends Coordinator {

  private static final Logger logger = new Logger(TeleopMode.class);

  private final InputPlayer inputPlayer = new InputPlayer();
  private InputRecorder inputRecorder;

  private final MarionetteJoystick driverJoystick = new MarionetteJoystick(0, inputPlayer, 0);
  private final MarionetteJoystick manipJoystick = new MarionetteJoystick(1, inputPlayer, 1);

  private final XboxController driver = new XboxController(driverJoystick);
  private final XboxController manip = new XboxController(manipJoystick);

  private final com._604robotics.robot2020.Robot2020 robot;

  private final DriveManager driveManager;
  private final IntakeManager intakeManager;
  // private final ShooterManager shooterManager;
  // private final AutoCenterManager autoCenterManager;

  private boolean autoCentering = false;
  private boolean aligned = false;

  private final Logger test = new Logger("Teleop");

  public TeleopMode(com._604robotics.robot2020.Robot2020 robot) {
    driver.leftStick.x.setDeadband(Calibration.TELEOP_DRIVE_DEADBAND);
    driver.leftStick.y.setDeadband(Calibration.TELEOP_DRIVE_DEADBAND);

    driver.leftStick.x.setFactor(Calibration.TELEOP_FACTOR);
    driver.leftStick.y.setFactor(Calibration.TELEOP_FACTOR);

    driver.rightStick.x.setDeadband(Calibration.TELEOP_DRIVE_DEADBAND);
    driver.rightStick.y.setDeadband(Calibration.TELEOP_DRIVE_DEADBAND);

    // driver.rightStick.x.setFactor(Calibration.TELEOP_FACTOR);
    driver.rightStick.x.setFactor(1); // WEIRD_WHY_?FES:RLJTH *ROHT guirg
    driver.rightStick.y.setFactor(Calibration.TELEOP_FACTOR);

    manip.leftStick.x.setDeadband(Calibration.TELEOP_MANIP_DEADBAND);
    manip.leftStick.y.setDeadband(Calibration.TELEOP_MANIP_DEADBAND);

    manip.leftStick.x.setFactor(Calibration.TELEOP_FACTOR);
    manip.leftStick.y.setFactor(Calibration.TELEOP_FACTOR);

    manip.rightStick.x.setDeadband(Calibration.TELEOP_MANIP_DEADBAND);
    manip.rightStick.y.setDeadband(Calibration.TELEOP_MANIP_DEADBAND);

    manip.rightStick.x.setFactor(Calibration.TELEOP_FACTOR);
    manip.rightStick.y.setFactor(Calibration.TELEOP_FACTOR);

    this.robot = robot;

    driveManager = new DriveManager();
    intakeManager = new IntakeManager();
    // shooterManager = new ShooterManager();
    // autoCenterManager = new AutoCenterManager();
  }

  // <editor-fold desc="Getting Controller Values"
  private double driverLeftJoystickY = 0.0;
  private double driverLeftJoystickX = 0.0;
  private double driverLeftTrigger = 0.0;

  private boolean driverLeftJoystickButton = false;
  private boolean driverLeftTriggerButton = false;
  private boolean driverLeftBumper = false;

  private double driverRightJoystickY = 0.0;
  private double driverRightJoystickX = 0.0;
  private double driverRightTrigger = 0.0;

  private boolean driverRightJoystickButton = false;
  private boolean driverRightTriggerButton = false;
  private boolean driverRightBumper = false;

  private boolean driverBack = false;
  private boolean driverStart = false;
  private boolean driverA = false;
  private boolean driverB = false;
  private boolean driverX = false;
  private boolean driverY = false;

  private boolean driverDPad = false;

  private double manipLeftJoystickY = 0.0;
  private double manipLeftJoystickX = 0.0;
  private double manipLeftTrigger = 0.0;

  private boolean manipLeftJoystickButton = false;
  private boolean manipLeftTriggerButton = false;
  private boolean manipLeftBumper = false;

  private double manipRightJoystickY = 0.0;
  private double manipRightJoystickX = 0.0;
  private double manipRightTrigger = 0.0;

  private boolean manipRightJoystickButton = false;
  private boolean manipRightTriggerButton = false;
  private boolean manipRightBumper = false;

  private boolean manipBack = false;
  private boolean manipStart = false;
  private boolean manipA = false;
  private boolean manipB = false;
  private boolean manipX = false;
  private boolean manipY = false;
  private boolean manipDPad = false;

  private boolean hatchCollisionChecker;
  private boolean armCollisionChecker;
  // </editor-fold>

  public void startPlayback(InputRecording recording) {
    inputPlayer.startPlayback(recording);
  }

  public void stopPlayback() {
    inputPlayer.stopPlayback();
  }

  @Override
  protected void begin() {
    if (inputPlayer.isPlaying()) {
      logger.info("Playing back Marionette recording");
    }
    hatchCollisionChecker = false;
    armCollisionChecker = false;
  }

  @Override
  protected boolean run() {
    updateControls();
    process();
    return true;
  }

  @Override
  protected void end() {
    if (inputRecorder != null) {
      final InputRecorder oldInputRecorder = inputRecorder;
      inputRecorder = null;
    }
  }

  private void updateControls() {
    driverLeftJoystickY = driver.leftStick.y.get();
    driverLeftJoystickX = driver.leftStick.x.get();
    driverLeftTrigger = driver.triggers.left.get();

    driverLeftJoystickButton = driver.buttons.leftStick.get();
    driverLeftTriggerButton = driver.buttons.lt.get();
    driverLeftBumper = driver.buttons.lb.get();

    driverRightJoystickY = driver.rightStick.y.get();
    driverRightJoystickX = driver.rightStick.x.get();
    driverRightTrigger = driver.triggers.right.get();

    driverRightJoystickButton = driver.buttons.rightStick.get();
    driverRightTriggerButton = driver.buttons.rt.get();
    driverRightBumper = driver.buttons.rb.get();

    driverBack = driver.buttons.back.get();
    driverStart = driver.buttons.start.get();
    driverA = driver.buttons.a.get();
    driverB = driver.buttons.b.get();
    driverX = driver.buttons.x.get();
    driverY = driver.buttons.y.get();

    driverDPad = driver.dpad.pressed.get();

    manipLeftJoystickY = manip.leftStick.y.get();
    manipLeftJoystickX = manip.leftStick.x.get();
    manipLeftTrigger = manip.triggers.left.get();

    manipLeftJoystickButton = manip.buttons.leftStick.get();
    manipLeftTriggerButton = manip.buttons.lt.get();
    manipLeftBumper = manip.buttons.lb.get();

    manipRightJoystickY = manip.rightStick.y.get();
    manipRightJoystickX = manip.rightStick.x.get();
    manipRightTrigger = manip.triggers.right.get();

    manipRightJoystickButton = manip.buttons.rightStick.get();
    manipRightTriggerButton = manip.buttons.rt.get();
    manipRightBumper = manip.buttons.rb.get();

    manipBack = manip.buttons.back.get();
    manipStart = manip.buttons.start.get();
    manipA = manip.buttons.a.get();
    manipB = manip.buttons.b.get();
    manipX = manip.buttons.x.get();
    manipY = manip.buttons.y.get();

    manipDPad = manip.dpad.pressed.get();
  }

  private void process() {
    driveManager.run();
    // shooterManager.run();
    intakeManager.run();
  }

  private class DriveManager {
    private final Swerve.OpenLoop openLoop;
    private final Swerve.Idle idle;

    // private final AutoCenterMacro autoCenterMacro;

    private CurrentDrive currentDrive;
    private CurrentDrive selectedDrive;
    private Toggle inverted;

    public DriveManager() {
      idle = robot.drive.new Idle();
      openLoop = robot.drive.new OpenLoop();

      // autoCenterMacro = new AutoCenterMacro(arcade, robot.limelight, false);

      currentDrive = CurrentDrive.OPENLOOP;
      inverted = new Toggle(false);
    }

    public void run() {
      double leftX = -driver.leftStick.x.get();
      double leftY = -driver.leftStick.y.get();
      double rightX = driver.rightStick.x.get();

      if (driverLeftJoystickButton) {
        leftX *= 0.8;
        leftY *= 0.8;
        rightX *= 0.8;
      }

      inverted.update(driverLeftBumper);
      if (inverted.isInOnState()) { // Flip values if xbox inverted
        leftX *= -1;
        leftY *= -1;
      }

      // // Get Dashboard option for drive
      // switch (robot.drive.driveMode.get()) {
      //   case OFF:
      //     currentDrive = CurrentDrive.IDLE;
      //     selectedDrive = CurrentDrive.IDLE;
      //     break;
      //   case ARCADE:
      //     currentDrive = CurrentDrive.ARCADE;
      //     selectedDrive = CurrentDrive.ARCADE;
      //     break;
      //   case TANK:
      //     currentDrive = CurrentDrive.TANK;
      //     selectedDrive = CurrentDrive.TANK;
      //     break;
      //   case DYNAMIC:
      //     // Dynamic Drive mode detection logic
      //     if (currentDrive == CurrentDrive.TANK) {
      //       if (Math.abs(rightY) <= 0.2 && Math.abs(rightX) > 0.3) {
      //         currentDrive = CurrentDrive.ARCADE;
      //         selectedDrive = CurrentDrive.ARCADE;
      //       }
      //     } else { // currentDrive == CurrentDrive.ARCADE
      //       if (Math.abs(rightX) <= 0.2 && Math.abs(rightY) > 0.3) {
      //         currentDrive = CurrentDrive.TANK;
      //       }
      //     }
      //     break;
      //   default:
      //     System.out.println("This should never happen!");
      //     System.out.println("Current value is:" + robot.drive.driveMode.get());
      // }

      switch (currentDrive) {
        case IDLE:
          idle.activate();
          break;
        case OPENLOOP:
          openLoop.xPower.set(leftX * 0.2);
          openLoop.yPower.set(leftY * 0.2);
          openLoop.rotPower.set(rightX * 0.2);

          openLoop.activate();
          break;
      }
    }
  }

  private class IntakeManager {
    private final Intake.Suck suck;
    private final Intake.Idle idle;
    private final IntakeDeploy.Deploy deploy;
    private final IntakeDeploy.Retract retract;
    private final Tower.AntiJam antiJam;
    private final Revolver.Intake revolve;
    private final AntiJamRoller.AntiJam roller;

    public IntakeManager() {
      suck = robot.intake.suck;
      idle = robot.intake.idle;
      deploy = robot.intakeDeploy.deploy;
      retract = robot.intakeDeploy.retract;
      antiJam = robot.tower.antiJam;
      revolve = robot.revolver.intake;
      roller = robot.antiJamRoller.antiJam;
    }

    public void run() {
      if (driverRightTrigger >= 0.05) {
        deploy.activate();
        suck.activate();
        revolve.activate();
        roller.activate();
        antiJam.activate();
      } else {
        retract.activate();
        idle.activate();
      }
    }
  }

  public enum CurrentDrive {
    IDLE,
    OPENLOOP,
  }
}
