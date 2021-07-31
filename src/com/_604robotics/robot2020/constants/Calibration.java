package com._604robotics.robot2020.constants;

import com._604robotics.quixsam.mathematics.DoubleInterpolatableTreeMap;
import com._604robotics.robotnik.prefabs.auto.TrackerConstants;
import com._604robotics.robotnik.utils.annotations.Unreal;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class Calibration {

  public static final double TELEOP_DRIVE_DEADBAND = 0.3;
  public static final double TELEOP_MANIP_DEADBAND = 0.2;
  public static final double TELEOP_FACTOR = -1;

  /* Climber Calibration */
  public static final class Climber {
    public static final double CLIMB_SPEED = -0.25;
    public static final double RETRACT_SPEED = 0.25;
  }

  /* Tower Calibration */
  public static final class Tower {
    public static final double EMPTY_SPEED = 1;
    public static final double INTAKE_SPEED = 1;
    public static final double ANTI_JAM_SPEED = -0.15;
  }

  /* Revolver Calibration */
  public static final class Revolver {
    // public static final double EMPTY_SPEED = 0.75;
    public static final double EMPTY_SPEED = -0.15;
    public static final double INTAKE_SPEED = 0.5;
    public static final double ANTI_JAM_SPEED = -1;
  }

  /* Anti Jam Roller Calibration */
  public static final class AntiJamRoller {
    public static final double ANTI_JAM_SPEED = -0.25;
  }

  /* Intake Calibration */
  public static final class Intake {
    public static final double INTAKE_SPEED = 0.75;
    public static final double ANTI_JAM_SPEED = -1;
  }

  /* Shooter Calibration */
  @Unreal("Get values from characterization.")
  public static final class Shooter {
    /* Pullies */
    public static final int DRIVING_TEETH = 36;
    public static final int DRIVEN_TEETH = 18;

    /* Velocity PD Controller */
    public static final double kP = 0;
    public static final double kD = 0; // Multiplying by 0.02 to time parameterize it.

    /* Feedforward */
    public static final double Ks = 0.547;
    public static double Kv = 0.0082;

    public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Ks, Kv);
  }

  /* Feeder Calibration */
  @Unreal("Get values from characterization.")
  public static final class Feeder {
    /* Velocity PD Controller */
    public static final double kP = 0.0015;
    public static final double kD = 0; // Multiplying by 0.02 to time parameterize it.

    /* Feedforward */
    public static final double Ks = 0.132;
    public static final double Kv = 0.0201;

    public static final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Ks, Kv);
  }

  public static final class Drive {
    public static final boolean GYRO_REVERSED = false;
    public static final double TRACK_WIDTH = 0.61595;
    public static final double WHEEL_DIAMETER = 0.1524;
    public static final double DISTANCE_PER_ROTATION = (WHEEL_DIAMETER * Math.PI);
    public static final double DISTANCE_PER_COUNT = DISTANCE_PER_ROTATION / 2048;

    public static final double SLOW_ROTATION_MODIFIER = 0.6;
  }

  public static final class AutoAlign {
    public static final double kP = 0.06;
    public static final double kD = 0.0;
    public static final double ABSOLUTE_TOLERANCE_OUTER_GOAL = 0.005;
    public static final double ABSOLUTE_TOLERANCE_INNER_GOAL = 0.005;
  }

  /* Auton Calibration */
  @Unreal("Get values from characterization.")
  public static final class Auto {
    public static final double RAMSETE_B = 2.0;
    public static final double RAMSETE_ZETA = 0.7;
    public static final double KS_VOLTS = 0.19;
    public static final double KV_VOLT_SECONDS_PER_METER = 2.47;
    public static final double KA_VOLT_SECONDS_SQUARED_PER_METER = 0.251;
    // public static final double KP_DRIVE_VELCOTIY = 2.52;
    public static final double KP_DRIVE_VELCOTIY = 0.6;

    public static final double MAX_SPEED_METERS_PER_SECOND = 4; // 10
    public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 2; // 1.5
    public static final double MAX_CENTRIPETAL_ACCELERATION_RADIANS_PER_SECOND_SQUARED = 2; // 1.5

    public static final TrackerConstants TRACKER_CONSTANTS =
        new TrackerConstants(
            new SimpleMotorFeedforward(
                KS_VOLTS, KV_VOLT_SECONDS_PER_METER, KA_VOLT_SECONDS_SQUARED_PER_METER),
            KP_DRIVE_VELCOTIY,
            RAMSETE_B,
            RAMSETE_ZETA,
            MAX_SPEED_METERS_PER_SECOND,
            MAX_ACCELERATION_METERS_PER_SECOND_SQUARED);

    public static DoubleInterpolatableTreeMap<Double> XDISTACEBALLCAM =
        new DoubleInterpolatableTreeMap<>();
  }

  public static final class Limelight {
    // :5800 includes framerate and crosshair :5802 does not :5801 is the control interface
    public static final String LIMELIGHT_URL = "http://10.6.4.101:5802";
    public static final int LIMELIGHT_FPS = 90; // Default: Uncapped (Normally around 90)
    public static final int LIMELIGHT_RES_X = 320; // Default: 320
    public static final int LIMELIGHT_RES_Y = 240; // Default: 240

    @Unreal("Needs to be measured")
    public static final double LIMELIGHT_HEIGHT =
        24.5; // Height of limelight from the ground in inches

    @Unreal("Needs to be measured")
    public static final double LIMELIGHT_ANGLE =
        80; // Angle of the limelight relative to the plane of the robot in degrees
    // If negative, pointed down, positive is pointed up
    @Unreal("Needs to be measured")
    public static final double TARGET_HEIGHT =
        28; // Height of the center of the vision target in inches

    public static final double LIMELIGHT_ANGLE_TOLERANCE = 0.005;
    public static final double LIMELIGHT_DIST_TOLERANCE = 1;
    public static final int LIMELIGHT_DRIVER_PIPE =
        9; // The pipeline to use for driver usage, NOT tracking
    public static final int LIMELIGHT_VISION_PIPE = 0; // Pipeline for vision tracking by default
  }

  /* Marionette */
  public static final boolean AUTO_APPEND_TIMESTAMP = true;
  public static final String CUSTOM_PRIMARY = "single00.switchLeft.marionette";
  public static final String CUSTOM_SECONDARY = "half00.switchRight.marionette";
}