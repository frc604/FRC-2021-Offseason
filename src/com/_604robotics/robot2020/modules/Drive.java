package com._604robotics.robot2020.modules;

import com._604robotics.robot2020.constants.Calibration;
import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.DashboardManager;
import com._604robotics.robotnik.Input;
import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.Output;
import com._604robotics.robotnik.prefabs.controller.ExtendablePIDController;
import com._604robotics.robotnik.prefabs.devices.ADIS16470;
import com._604robotics.robotnik.prefabs.devices.NEOEncoder;
import com._604robotics.robotnik.prefabs.modules.Shifter;
import com._604robotics.robotnik.prefabs.motorcontrol.Motor;
import com._604robotics.robotnik.prefabs.motorcontrol.QuixSparkMAX;
import com._604robotics.robotnik.prefabs.motorcontrol.controllers.SparkPID;
import com._604robotics.robotnik.prefabs.motorcontrol.gearing.GearRatio;
import com._604robotics.robotnik.prefabs.motorcontrol.wpilibj.DifferentialDrive;
import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;

public class Drive extends Module {
  /* Motors */
  private final QuixSparkMAX frontLeft = new QuixSparkMAX(11, "Front Left Motor", Motor.kNEO, this);
  private final QuixSparkMAX rearLeft = new QuixSparkMAX(3, "Rear Left Motor", Motor.kNEO, this);

  private final QuixSparkMAX frontRight =
      new QuixSparkMAX(5, "Front Right Motor", Motor.kNEO, this);
  private final QuixSparkMAX rearRight = new QuixSparkMAX(18, "Rear Right Motor", Motor.kNEO, this);

  private final DifferentialDrive robotDrive = new DifferentialDrive(frontLeft, frontRight);

  /* Encoders */
  public final Shifter shifter = new Shifter(4, 5);

  private final GearRatio ratio = new GearRatio(10000, 95625); // Assuming auton is run in high gear

  private final NEOEncoder encoderLeft = new NEOEncoder(frontLeft, ratio);
  private final NEOEncoder encoderRight = new NEOEncoder(frontRight, ratio);

  private final Encoder externalEncoderLeft = new Encoder(1, 0, false, EncodingType.k1X);
  private final Encoder externalEncoderRight = new Encoder(3, 2, true, EncodingType.k1X);

  /* PID */
  public final SparkPID leftPID;
  public final SparkPID rightPID;

  public final ExtendablePIDController leftController;
  public final ExtendablePIDController rightController;

  /* Gyro */
  private final AnalogGyro horizGyro = new AnalogGyro(0);
  private final ADIS16470 imu = ADIS16470.getInstance();

  /* Kinematics */
  public DifferentialDriveKinematics driveKinematics =
      new DifferentialDriveKinematics(Calibration.Drive.TRACK_WIDTH);

  public DifferentialDriveOdometry driveOdometry =
      new DifferentialDriveOdometry(
          Rotation2d.fromDegrees(getHeading()),
          new Pose2d(Units.feetToMeters(1), Units.feetToMeters(0), Rotation2d.fromDegrees(180)));

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

  public final Output<Double> leftClickRate =
      addOutput("leftClickRate", externalEncoderLeft::getRate);
  public final Output<Double> rightClickRate =
      addOutput("rightClickRate", externalEncoderRight::getRate);

  public final Output<Double> robotHeading = addOutput("Robot Heading", this::getHeading);
  public final Output<Double> robotX = addOutput("Robot X Position", this::getX);
  public final Output<Double> robotY = addOutput("Robot Y Position", this::getY);

  public final Output<Double> leftDistance = addOutput("leftDistance", encoderLeft::getPosition);
  public final Output<Double> rightDistance = addOutput("rightDistance", encoderRight::getPosition);

  /* Choosers */
  public final Output<DriveMode> driveMode;

  /* Auton Methods */
  public double getHeading() {
    var angle = -imu.getAngle() * (Calibration.Drive.GYRO_REVERSED ? -1.0 : 1.0);
    return Math.IEEEremainder(angle, 360);
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        externalEncoderLeft.getRate(), externalEncoderRight.getRate());
  }

  public ChassisSpeeds getChassisSpeeds() {
    return driveKinematics.toChassisSpeeds(getWheelSpeeds());
  }

  public void enableMotorSafety() {
    robotDrive.setSafetyEnabled(true);
  }

  public void disableMotorSafety() {
    robotDrive.setSafetyEnabled(false);
  }

  public void setIdleMode(IdleMode mode) {
    frontLeft.controller.setIdleMode(mode);
    rearLeft.controller.setIdleMode(mode);
    frontRight.controller.setIdleMode(mode);
    rearRight.controller.setIdleMode(mode);
  }

  /* Pose Getters */
  public Pose2d getPose() {
    return driveOdometry.getPoseMeters();
  }

  public double getHeadingDegrees() {
    return getPose().getRotation().getDegrees();
  }

  public double getX() {
    return getPose().getTranslation().getX();
  }

  public double getY() {
    return getPose().getTranslation().getY();
  }

  /* Odometry Methods */
  public void updateOdometry() {
    driveOdometry.update(
        Rotation2d.fromDegrees(getHeading()),
        externalEncoderLeft.getDistance(),
        externalEncoderRight.getDistance());
  }

  public void zeroOdometry() {
    encoderLeft.zero();
    encoderRight.zero();
    externalEncoderLeft.reset();
    externalEncoderRight.reset();
    driveOdometry.resetPosition(
        new Pose2d(Units.feetToMeters(0), Units.feetToMeters(0), Rotation2d.fromDegrees(180.0)),
        Rotation2d.fromDegrees(getHeading()));
  }

  public void zeroOdometry(Pose2d pose) {
    encoderLeft.zero();
    encoderRight.zero();
    externalEncoderLeft.reset();
    externalEncoderRight.reset();
    driveOdometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
  }

  public synchronized void resetSensors() {
    encoderLeft.zero();
    encoderRight.zero();
    externalEncoderLeft.reset();
    externalEncoderRight.reset();
    horizGyro.reset();
    imu.reset();
  }

  /* Drive Actions */
  public class Idle extends Action {
    public Idle() {
      super(Drive.this, Idle.class);
    }

    @Override
    public void run() {
      robotDrive.stopMotor();
    }
  }

  public class TankDrive extends Action {
    public final Input<Double> leftPower;
    public final Input<Double> rightPower;
    public final boolean squared;

    public TankDrive() {
      this(0, 0, true);
    }

    public TankDrive(boolean squared) {
      this(0, 0, squared);
    }

    public TankDrive(double defaultLeftPower, double defaultRightPower) {
      this(defaultLeftPower, defaultRightPower, true);
    }

    public TankDrive(double defaultLeftPower, double defaultRightPower, boolean squared) {
      super(Drive.this, TankDrive.class);
      leftPower = addInput("leftPower", defaultLeftPower, true);
      rightPower = addInput("rightPower", defaultRightPower, true);
      this.squared = squared;
    }

    @Override
    public void run() {
      if (leftPower.get() > 1
          || leftPower.get() < -1
          || rightPower.get() > 1
          || rightPower.get() < -1) {
        System.out.println("L" + leftPower.get() + "R" + rightPower.get());
      }
      robotDrive.tankDrive(leftPower.get(), rightPower.get(), squared);
    }
  }

  public class SparkDrivePID extends Action {
    public final Input<Double> leftVel;
    public final Input<Double> rightVel;
    public final Input<Double> leftFF;
    public final Input<Double> rightFF;

    public SparkDrivePID() {
      this(0, 0, 0, 0);
    }

    public SparkDrivePID(
        double defaultLeftVel,
        double defaultRightVel,
        double defaultLeftFF,
        double defaultRightFF) {
      super(Drive.this, SparkPID.class);
      leftVel = addInput("leftVolts", defaultLeftVel, true);
      rightVel = addInput("rightVolts", defaultRightVel, true);

      leftFF = addInput("leftFF", defaultLeftFF, true);
      rightFF = addInput("rightFF", defaultRightFF, true);
    }

    @Override
    public void run() {
      leftPID.setSetpointVelocity(leftVel.get(), leftFF.get());
      rightPID.setSetpointVelocity(rightVel.get(), rightFF.get());
    }
  }

  public class DrivePID extends Action {
    public final Input<Double> leftVel;
    public final Input<Double> rightVel;
    public final Input<Double> leftFF;
    public final Input<Double> rightFF;

    public DrivePID() {
      this(0, 0, 0, 0);
    }

    public DrivePID(
        double defaultLeftVel,
        double defaultRightVel,
        double defaultLeftFF,
        double defaultRightFF) {
      super(Drive.this, DrivePID.class);
      leftVel = addInput("leftVelocity", defaultLeftVel, true);
      rightVel = addInput("rightVelocity", defaultRightVel, true);

      leftFF = addInput("leftFF", defaultLeftFF, true);
      rightFF = addInput("rightFF", defaultRightFF, true);
    }

    @Override
    public void run() {
      leftController.setSetpoint(leftVel.get());
      leftController.setF(leftFF.get());
      rightController.setSetpoint(rightVel.get());
      rightController.setF(rightFF.get());

      leftController.enable();
      rightController.enable();
    }

    @Override
    public void end() {
      leftController.disable();
      rightController.disable();
    }
  }

  public class TankDriveVolts extends Action {
    public final Input<Double> leftVolts;
    public final Input<Double> rightVolts;

    public TankDriveVolts() {
      this(0, 0);
    }

    public TankDriveVolts(double defaultLeftVolts, double defaultRightVolts) {
      super(Drive.this, TankDriveVolts.class);
      leftVolts = addInput("leftVolts", defaultLeftVolts, true);
      rightVolts = addInput("rightVolts", defaultRightVolts, true);
    }

    @Override
    public void run() {
      frontLeft.setVoltage(leftVolts.get());
      frontRight.setVoltage(-rightVolts.get());
    }
  }

  public class ArcadeDrive extends Action {
    public final Input<Double> movePower;
    public final Input<Double> rotatePower;
    public final boolean squared;

    public ArcadeDrive() {
      this(0, 0, true);
    }

    public ArcadeDrive(boolean squared) {
      this(0, 0, squared);
    }

    public ArcadeDrive(double defaultMovePower, double defaultRotPower) {
      this(defaultMovePower, defaultRotPower, true);
    }

    public ArcadeDrive(double defaultMovePower, double defaultRotPower, boolean squared) {
      super(Drive.this, ArcadeDrive.class);
      movePower = addInput("movePower", defaultMovePower, true);
      rotatePower = addInput("rotatePower", defaultRotPower, true);
      this.squared = squared;
    }

    @Override
    public void run() {
      robotDrive.arcadeDrive(movePower.get(), rotatePower.get(), squared);
    }
  }

  public final Action idle = new Idle();

  public Drive() {
    super(Drive.class);

    frontLeft.resetParams();
    frontRight.resetParams();
    rearLeft.resetParams();
    rearRight.resetParams();

    robotDrive.setRightSideInverted(false);

    horizGyro.setSensitivity(0.00665);
    imu.calibrate();

    /* Follower */
    frontRight.setInverted(true);

    rearLeft.follow(frontLeft, false);
    rearRight.follow(frontRight, false);

    /* Encoders */
    encoderLeft.setInverted(false);
    encoderRight.setInverted(false);

    encoderLeft.setdistancePerRotation(Calibration.Drive.DISTANCE_PER_ROTATION);
    encoderRight.setdistancePerRotation(Calibration.Drive.DISTANCE_PER_ROTATION);

    externalEncoderLeft.setDistancePerPulse(Calibration.Drive.DISTANCE_PER_COUNT);
    externalEncoderRight.setDistancePerPulse(Calibration.Drive.DISTANCE_PER_COUNT);

    /* Current Limits */
    frontLeft.setCurrentLimit(75);
    frontLeft.enableCurrentLimit(true);

    frontRight.setCurrentLimit(75);
    frontRight.enableCurrentLimit(true);

    rearLeft.setCurrentLimit(75);
    rearLeft.enableCurrentLimit(true);

    rearRight.setCurrentLimit(75);
    rearRight.enableCurrentLimit(true);

    /* Braking */
    setIdleMode(IdleMode.kCoast);

    /* Deadband */
    robotDrive.setDeadband(0.04);

    /* Choosers */
    driveMode =
        DashboardManager.getInstance()
            .registerEnumOutput("Drive Mode Chooser", DriveMode.ARCADE, DriveMode.class, this);

    /* Init */
    resetSensors();
    setDefaultAction(idle);

    leftPID = new SparkPID(frontLeft, Calibration.Auto.KP_DRIVE_VELCOTIY, 0.0, 0.0);
    rightPID = new SparkPID(frontRight, Calibration.Auto.KP_DRIVE_VELCOTIY, 0.0, 0.0);

    leftController =
        new ExtendablePIDController(
            Calibration.Auto.KP_DRIVE_VELCOTIY,
            0.0,
            0.0,
            externalEncoderLeft::getRate,
            frontLeft::setVoltage,
            0.01);
    rightController =
        new ExtendablePIDController(
            Calibration.Auto.KP_DRIVE_VELCOTIY,
            0.0,
            0.0,
            externalEncoderRight::getRate,
            frontRight::setVoltage,
            0.01);

    leftController.setOutputRange(-12, 12);
    rightController.setOutputRange(-12, 12);

    /* Burning 🔥 */
    frontLeft.burnFlashConditionally(false);
    frontRight.burnFlashConditionally(false);
    rearLeft.burnFlashConditionally(false);
    rearRight.burnFlashConditionally(false);
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
