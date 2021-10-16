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
import com._604robotics.robotnik.prefabs.swerve.QuixSwerveDriveOdometry;
import com._604robotics.robotnik.prefabs.swerve.QuixSwerveModule;
import com._604robotics.robotnik.prefabs.swerve.QuixSwerveModuleState;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Units;

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
    new MotorControllerPIDConfig(0.6, 0.0, 12.0),
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
    new MotorControllerPIDConfig(0.6, 0.0, 12.0),
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
    new MotorControllerPIDConfig(0.6, 0.0, 12.0),
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
    new MotorControllerPIDConfig(0.6, 0.0, 12.0),
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
  public QuixSwerveDriveKinematics kinematics = new QuixSwerveDriveKinematics(
    frontLeft.getPosition(),
    frontRight.getPosition(),
    rearLeft.getPosition(),
    rearRight.getPosition()
  );

  /* Odometry */
  public QuixSwerveDriveOdometry odometry = new QuixSwerveDriveOdometry(
    kinematics,
    getHeading(),
    new Pose2d(Units.feetToMeters(1), Units.feetToMeters(0), Rotation2d.fromDegrees(180))
  );

  /* Outputs */
  private final BuiltInAccelerometer accel = new BuiltInAccelerometer();
  public final Output<Double> xAccel = addOutput("X accel", accel::getX);
  public final Output<Double> yAccel = addOutput("Y accel", accel::getY);
  public final Output<Double> zAccel = addOutput("Z accel", accel::getZ);

  public final Output<Double> imuXAccel = addOutput("IMU X accel", imu::getXAccel);
  public final Output<Double> imuYAccel = addOutput("IMU Y accel", imu::getYAccel);

  public final Output<Double> gyroAngle = addOutput("gyroAngle", imu::getAngle);

  public final Output<Double> frontLeftAngle = addOutput("Front Left Angle", frontLeft::getAngle);
  public final Output<Double> frontRightAngle = addOutput("Front Right Angle", frontRight::getAngle);
  public final Output<Double> rearLeftAngle = addOutput("Rear Left Angle", rearLeft::getAngle);
  public final Output<Double> rearRightAngle = addOutput("Rear Right Angle", rearRight::getAngle);

  public final Output<Double> absfrontLeftAngle = addOutput("Abs Front Left Angle", frontLeft::getAbsEncoderAngle);
  public final Output<Double> absfrontRightAngle = addOutput("Abs Front Right Angle", frontRight::getAbsEncoderAngle);
  public final Output<Double> absrearLeftAngle = addOutput("Abs Rear Left Angle", rearLeft::getAbsEncoderAngle);
  public final Output<Double> absrearRightAngle = addOutput("Abs Rear Right Angle", rearRight::getAbsEncoderAngle);

  public final Output<Double> robotHeading = addOutput("Robot Heading", this::getHeadingDegrees);
  public final Output<Double> robotX = addOutput("Robot X Position", this::getX);
  public final Output<Double> robotY = addOutput("Robot Y Position", this::getY);
  
  
  
  /* Auton Methods */
  public Rotation2d getHeading() {
    return (Calibration.Drive.GYRO_REVERSED) ? Rotation2d.fromDegrees(360 - imu.getAngle()) : Rotation2d.fromDegrees(imu.getAngle());
    // var angle = -imu.getAngle() * (Calibration.Drive.GYRO_REVERSED ? -1.0 : 1.0);
    // return Rotation2d.fromDegrees(Math.IEEEremainder(angle, 360));
  }

  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  public double getHeadingDegrees() {
    return getHeading().getDegrees();
  }

  public double getRawHeadingDegrees() {
    return -imu.getAngle();
  }

  public double getAngularVelDegrees() {
    return imu.getRate() * (Calibration.Drive.GYRO_REVERSED ? -1.0 : 1.0);
  }

  public double getX() {
    return getPose().getTranslation().getX();
  }

  public double getY() {
    return getPose().getTranslation().getY();
  }

  public void zeroOdometry(Pose2d pose) {
    odometry.resetPosition(pose, getHeading());
  }

  public void zeroGyro() {
    imu.calibrate();
    imu.reset();
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
      kinematics.toSwerveModuleStates(
          fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                              translation.getX(), 
                              translation.getY(), 
                              rotation, 
                              getHeading()
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
      drive(translation, rotation, true);
    }
  }

  public class AutoAngle extends Action {
    public final Input<Double> xPower;
    public final Input<Double> yPower;
    public final Input<Double> desiredAngularVel;

    public AutoAngle() {
      this(0.0, 0.0, 0.0);
    }

    public AutoAngle(double defaultXPower, double defaultYPower, double defaultDesiredAngularVel) {
      super(Swerve.this, AutoAngle.class);
      xPower = addInput("XPower", defaultXPower, true);
      yPower = addInput("YPower", defaultYPower, true);
      desiredAngularVel = addInput("RotHeading", defaultDesiredAngularVel, true);
    }

    @Override
    public void run() {
      Translation2d translation = new Translation2d(yPower.get(), xPower.get()).times(Calibration.Drive.MAX_DRIVE_VELOCITY);
      drive(translation, desiredAngularVel.get(), true);
    }
  }

  public class Auto extends Action {
    public final Input<QuixSwerveModuleState[]> moduleStates;
    public final Input<double[]> feedforwards;

    public Auto() {
      this(new QuixSwerveModuleState[4], new double[4]);
    }

    public Auto(QuixSwerveModuleState[] defaultModuleStates, double[] defaultFeedforwards) {
      super(Swerve.this, Auto.class);
      moduleStates = addInput("ModuleStates", defaultModuleStates, true);
      feedforwards = addInput("Feedforwards", defaultFeedforwards, true);
    }

    @Override
    public void run() {
      var desiredStates = moduleStates.get();
      var desiredFeedforwards = feedforwards.get();
      // QuixSwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, Calibration.Drive.MAX_DRIVE_VELOCITY);
      for(QuixSwerveModule module : modules){ 
        // module.setDesiredStateClosedLoop(desiredStates[module.getID()], desiredFeedforwards[module.getID()]);
      }
    }
  }

  public final Action idle = new Idle();

  public Swerve() {
    super(Swerve.class);

    for (QuixSwerveModule module : modules) {
      module.zeroToAbsPosition();
    }


    horizGyro.setSensitivity(0.00665);
    imu.calibrate();

    /* Choosers */
    // driveMode =
    //     DashboardManager.getInstance()
    //         .registerEnumOutput("Drive Mode Chooser", DriveMode.ARCADE, DriveMode.class, this);

    /* Init */
    setDefaultAction(idle);

    /* Burning 🔥 */
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
