package com._604robotics.robot2020.modules;

import com._604robotics.robot2020.constants.Calibration;
import com._604robotics.robot2020.constants.Ports;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.devices.ADIS16470;
import com._604robotics.robotnik.prefabs.motorcontrol.controllers.MotorControllerPIDConfig;
import com._604robotics.robotnik.prefabs.swerve.QuixFalconSwerveModule;
import com._604robotics.robotnik.prefabs.swerve.QuixSwerveDriveKinematics;
import com._604robotics.robotnik.prefabs.swerve.QuixSwerveModule;
import com._604robotics.robotnik.prefabs.swerve.QuixSwerveModuleState;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;

public class Swerve extends Module {
  /* Modules */
  private final QuixFalconSwerveModule frontLeft = QuixFalconSwerveModule.createModule(
    "Front Left",
    0,
    this,
    Calibration.Drive.FRONT_LEFT_POS,
    Ports.MODULE_0_DRIVE,
    Ports.MODULE_0_STEER,
    Ports.MODULE_0_ABS_ENCODER,
    false,
    false,
    new MotorControllerPIDConfig(0.0, 0.0, 0.0),
    new MotorControllerPIDConfig(0.2, 0.0, 1.0),
    Calibration.Drive.DRIVE_RATIO,
    Calibration.Drive.STEERING_RATIO,
    76.64,
    Calibration.Drive.WHEEL_DIAMETER,
    Calibration.Drive.MAX_DRIVE_VELOCITY
  );

  private final QuixFalconSwerveModule frontRight = QuixFalconSwerveModule.createModule(
    "Front Right",
    1,
    this,
    Calibration.Drive.FRONT_RIGHT_POS,
    Ports.MODULE_1_DRIVE,
    Ports.MODULE_1_STEER,
    Ports.MODULE_1_ABS_ENCODER,
    false,
    false,
    new MotorControllerPIDConfig(0.0, 0.0, 0.0),
    new MotorControllerPIDConfig(0.2, 0.0, 1.0),
    Calibration.Drive.DRIVE_RATIO,
    Calibration.Drive.STEERING_RATIO,
    203.03,
    Calibration.Drive.WHEEL_DIAMETER,
    Calibration.Drive.MAX_DRIVE_VELOCITY
  );

  private final QuixFalconSwerveModule rearLeft = QuixFalconSwerveModule.createModule(
    "Rear Left",
    2,
    this,
    Calibration.Drive.REAR_LEFT_POS,
    Ports.MODULE_2_DRIVE,
    Ports.MODULE_2_STEER,
    Ports.MODULE_2_ABS_ENCODER,
    false,
    false,
    new MotorControllerPIDConfig(0.0, 0.0, 0.0),
    new MotorControllerPIDConfig(0.2, 0.0, 1.0),
    Calibration.Drive.DRIVE_RATIO,
    Calibration.Drive.STEERING_RATIO,
    211.38,
    Calibration.Drive.WHEEL_DIAMETER,
    Calibration.Drive.MAX_DRIVE_VELOCITY
  );

  private final QuixFalconSwerveModule rearRight = QuixFalconSwerveModule.createModule(
    "Rear Right",
    3,
    this,
    Calibration.Drive.REAR_RIGHT_POS,
    Ports.MODULE_3_DRIVE,
    Ports.MODULE_3_STEER,
    Ports.MODULE_3_ABS_ENCODER,
    false,
    false,
    new MotorControllerPIDConfig(0.0, 0.0, 0.0),
    new MotorControllerPIDConfig(0.2, 0.0, 1.0),
    Calibration.Drive.DRIVE_RATIO,
    Calibration.Drive.STEERING_RATIO,
    93.25,
    Calibration.Drive.WHEEL_DIAMETER,
    Calibration.Drive.MAX_DRIVE_VELOCITY
  );

  private final QuixSwerveModule[] modules = {
    frontLeft,
    frontRight,
    rearLeft,
    rearRight
  };


  /* Gyro */
  private final AnalogGyro horizGyro = new AnalogGyro(0);
  private final ADIS16470 imu = ADIS16470.getInstance();

  /* Kinematics */
  public QuixSwerveDriveKinematics driveKinematics = new QuixSwerveDriveKinematics(
    frontLeft.getPosition(),
    frontRight.getPosition(),
    rearLeft.getPosition(),
    rearRight.getPosition()
  );


  /* Outputs */
  /*
  private final BuiltInAccelerometer accel = new BuiltInAccelerometer();
  public final Output<Double> xAccel = addOutput("X accel", accel::getX);
  public final Output<Double> yAccel = addOutput("Y accel", accel::getY);
  public final Output<Double> zAccel = addOutput("Z accel", accel::getZ);
  */

  public final Output<Double> xAccel = addOutput("X accel", imu::getXAccel);
  public final Output<Double> yAccel = addOutput("Y accel", imu::getYAccel);

  public final Output<Double> gyroAngle = addOutput("gyroAngle", imu::getAngle);

  public final Output<Double> frontLeftAngle = addOutput("Front Left Angle", frontLeft::getAngle);
  public final Output<Double> frontRightAngle = addOutput("Front Right Angle", frontRight::getAngle);
  public final Output<Double> rearLeftAngle = addOutput("Rear Left Angle", rearLeft::getAngle);
  public final Output<Double> rearRightAngle = addOutput("Rear Right Angle", rearRight::getAngle);

  public final Output<Double> absfrontLeftAngle = addOutput("Abs Front Left Angle", frontLeft::getAbsEncoderAngle);
  public final Output<Double> absfrontRightAngle = addOutput("Abs Front Right Angle", frontRight::getAbsEncoderAngle);
  public final Output<Double> absrearLeftAngle = addOutput("Abs Rear Left Angle", rearLeft::getAbsEncoderAngle);
  public final Output<Double> absrearRightAngle = addOutput("Abs Rear Right Angle", rearRight::getAbsEncoderAngle);

  // public final Output<Double> robotHeading = addOutput("Robot Heading", this::getHeading);
  // public final Output<Double> robotX = addOutput("Robot X Position", this::getX);
  // public final Output<Double> robotY = addOutput("Robot Y Position", this::getY);
  
  /* Auton Methods */
  public double getHeading() {
    var angle = -imu.getAngle() * (Calibration.Drive.GYRO_REVERSED ? -1.0 : 1.0);
    return Math.IEEEremainder(angle, 360);
  }

  public void setModuleStates(boolean openLoop, QuixSwerveModuleState... desiredStates) {
    QuixSwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Calibration.Drive.MAX_DRIVE_VELOCITY);
    
    for(QuixSwerveModule module : modules){
        if (openLoop) {
          module.setDesiredStateOpenLoop(desiredStates[module.getID()]);
        }
    }
  }  
  
  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    QuixSwerveModuleState[] swerveModuleStates =
      driveKinematics.toSwerveModuleStates(
          fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                              translation.getX(), 
                              translation.getY(), 
                              rotation, 
                              new Rotation2d(getHeading())
                          )
                          : new ChassisSpeeds(
                              translation.getX(), 
                              translation.getY(), 
                              rotation)
                          );
    QuixSwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Calibration.Drive.MAX_DRIVE_VELOCITY);
    setModuleStates(true, swerveModuleStates);
  }

  /* Drive Actions */
  public class Idle extends Action {
    public Idle() {
      super(Swerve.this, Idle.class);
    }

    @Override
    public void run() {
      setModuleStates(true,
        new QuixSwerveModuleState(0.0, new Rotation2d()),
        new QuixSwerveModuleState(0.0, new Rotation2d()),
        new QuixSwerveModuleState(0.0, new Rotation2d()),
        new QuixSwerveModuleState(0.0, new Rotation2d())
      );
    }
  }

  public class OpenLoop extends Action {
    public final Input<Double> xPower;
    public final Input<Double> yPower;
    public final Input<Double> rotPower;

    public OpenLoop() {
      this(0.0, 0.0, 0.0);
    }

    public OpenLoop(double defaultXPower, double defaultYPower, double defaultRotPower) {
      super(Swerve.this, OpenLoop.class);
      xPower = addInput("XPower", defaultXPower, true);
      yPower = addInput("YPower", defaultYPower, true);
      rotPower = addInput("RotPower", defaultRotPower, true);
    }

    @Override
    public void run() {
      Translation2d translation = new Translation2d(yPower.get(), xPower.get()).times(Calibration.Drive.MAX_DRIVE_VELOCITY);
      double rotation = rotPower.get() * Calibration.Drive.MAX_ANGULAR_VELOCITY;
      drive(translation, rotation, false);
    }
  }


  public final Action idle = new Idle();

  public Swerve() {
    super(Swerve.class);

    for (QuixSwerveModule module : modules) {
      module.zeroToAbsPosition();
    }

    // frontLeft.resetParams();
    // frontRight.resetParams();
    // rearLeft.resetParams();
    // rearRight.resetParams();

    // robotDrive.setRightSideInverted(false);

    // horizGyro.setSensitivity(0.00665);
    // imu.calibrate();

    // /* Follower */
    // frontRight.setInverted(true);

    // rearLeft.follow(frontLeft, false);
    // rearRight.follow(frontRight, false);

    // /* Encoders */
    // encoderLeft.setInverted(false);
    // encoderRight.setInverted(false);

    // encoderLeft.setdistancePerRotation(Calibration.Drive.DISTANCE_PER_ROTATION);
    // encoderRight.setdistancePerRotation(Calibration.Drive.DISTANCE_PER_ROTATION);

    // externalEncoderLeft.setDistancePerPulse(Calibration.Drive.DISTANCE_PER_COUNT);
    // externalEncoderRight.setDistancePerPulse(Calibration.Drive.DISTANCE_PER_COUNT);

    // /* Current Limits */
    // frontLeft.setCurrentLimit(75);
    // frontLeft.enableCurrentLimit(true);

    // frontRight.setCurrentLimit(75);
    // frontRight.enableCurrentLimit(true);

    // rearLeft.setCurrentLimit(75);
    // rearLeft.enableCurrentLimit(true);

    // rearRight.setCurrentLimit(75);
    // rearRight.enableCurrentLimit(true);

    // /* Braking */
    // setIdleMode(IdleMode.kCoast);

    // /* Deadband */
    // robotDrive.setDeadband(0.04);

    // /* Choosers */
    // driveMode =
    //     DashboardManager.getInstance()
    //         .registerEnumOutput("Drive Mode Chooser", DriveMode.ARCADE, DriveMode.class, this);

    // /* Init */
    // resetSensors();
    // setDefaultAction(idle);

    // leftPID = new SparkPID(frontLeft, Calibration.Auto.KP_DRIVE_VELCOTIY, 0.0, 0.0);
    // rightPID = new SparkPID(frontRight, Calibration.Auto.KP_DRIVE_VELCOTIY, 0.0, 0.0);

    // leftController =
    //     new ExtendablePIDController(
    //         Calibration.Auto.KP_DRIVE_VELCOTIY,
    //         0.0,
    //         0.0,
    //         externalEncoderLeft::getRate,
    //         frontLeft::setVoltage,
    //         0.01);
    // rightController =
    //     new ExtendablePIDController(
    //         Calibration.Auto.KP_DRIVE_VELCOTIY,
    //         0.0,
    //         0.0,
    //         externalEncoderRight::getRate,
    //         frontRight::setVoltage,
    //         0.01);

    // leftController.setOutputRange(-12, 12);
    // rightController.setOutputRange(-12, 12);

    // /* Burning 🔥 */
    // frontLeft.burnFlashConditionally(false);
    // frontRight.burnFlashConditionally(false);
    // rearLeft.burnFlashConditionally(false);
    // rearRight.burnFlashConditionally(false);
  }

  public enum DriveMode {
    OFF,
    IDLE,
    ARCADE,
    TANK,
    DYNAMIC,
    MANUAL
  }
}
