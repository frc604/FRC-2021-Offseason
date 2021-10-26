// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package com._604robotics.robotnik.prefabs.swerve;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

/** Represents the state of one swerve module. */
@SuppressWarnings("MemberName")
public class QuixSwerveModuleState implements Comparable<QuixSwerveModuleState> {
  /** Speed of the wheel of the module. */
  public double speedMetersPerSecond;

  /** Angle of the module. */
  public Rotation2d angle = Rotation2d.fromDegrees(0);

  /** Constructs a SwerveModuleState with zeros for speed and angle. */
  public QuixSwerveModuleState() {}

  /**
   * Constructs a SwerveModuleState.
   *
   * @param speedMetersPerSecond The speed of the wheel of the module.
   * @param angle The angle of the module.
   */
  public QuixSwerveModuleState(double speedMetersPerSecond, Rotation2d angle) {
    this.speedMetersPerSecond = speedMetersPerSecond;
    this.angle = angle;
  }

  /**
   * Compares two swerve module states. One swerve module is "greater" than the other if its speed
   * is higher than the other.
   *
   * @param o The other swerve module.
   * @return 1 if this is greater, 0 if both are equal, -1 if other is greater.
   */
  @Override
  @SuppressWarnings("ParameterName")
  public int compareTo(QuixSwerveModuleState o) {
    return Double.compare(this.speedMetersPerSecond, o.speedMetersPerSecond);
  }

  @Override
  public String toString() {
    return String.format(
        "SwerveModuleState(Speed: %.2f m/s, Angle: %s)", speedMetersPerSecond, angle);
  }

  /**
   * Minimize the change in heading the desired swerve module state would require by potentially
   * reversing the direction the wheel spins. If this is used with the PIDController class's
   * continuous input functionality, the furthest a wheel will ever rotate is 90 degrees.
   *
   * @param desiredState The desired state.
   * @param currentAngle The current module angle.
   * @return Optimized swerve module state.
   */
  public static QuixSwerveModuleState optimize(QuixSwerveModuleState desiredState, Rotation2d currentAngle) {
    double targetAngle = placeInScope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
    double targetSpeed = desiredState.speedMetersPerSecond;
    double delta = targetAngle - currentAngle.getDegrees();
    if (Math.abs(delta) > 90){
        targetSpeed = -targetSpeed;
        targetAngle = delta > 90 ? (targetAngle -= 180) : (targetAngle += 180);
    }        
    return new QuixSwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
  }

  private static double placeInScope(double currentAngle, double desiredAngle) {
    // Place the desired angle in the scope [360(n-1), 360(n)] of the current angle.
    double angle = Math.floor(currentAngle / 360.0) * 360.0 + mod(desiredAngle, 360);

    // Constraint the desired angle to be < 180 degrees from the current angle.
    if ((angle - currentAngle) > 180) {
        angle -= 360;
    } else if ((angle - currentAngle) < -180) {
        angle += 360;
    }

    return angle;
  }

  private static double mod(double a, double n) {
    return (a % n + n) % n;
  }
}