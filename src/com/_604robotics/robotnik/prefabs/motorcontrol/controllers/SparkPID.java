package com._604robotics.robotnik.prefabs.motorcontrol.controllers;

import com._604robotics.robotnik.prefabs.motorcontrol.QuixSparkMAX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.ControlType;

public class SparkPID {
  public CANPIDController pid;

  private int slot = 0;

  /**
   * A wrapper representing a PID controller running on a REV Robotics SparkMAX.
   *
   * @param spark The QuixSparkMAX({@link
   *     com._604robotics.robotnik.prefabs.motorcontrol.QuixSparkMAX}) object that the controller is
   *     running on.
   * @param Kp The initial proportional gain.
   * @param Ki The initial integral gain.
   * @param KD The initial derivative gain.
   */
  public SparkPID(QuixSparkMAX spark, double Kp, double Ki, double Kd) {
    this.pid = spark.controller.getPIDController();

    this.pid.setP(Kp);
    this.pid.setI(Ki);
    this.pid.setD(Kd);
  }

  /**
   * A wrapper representing a PID controller running on a REV Robotics SparkMAX.
   *
   * @param spark The QuixSparkMAX({@link
   *     com._604robotics.robotnik.prefabs.motorcontrol.QuixSparkMAX}) object that the controller is
   *     running on.
   * @param slot The slot on the Spark to put the controller on.
   * @param Kp The initial proportional gain.
   * @param Ki The initial integral gain.
   * @param KD The initial derivative gain.
   */
  public SparkPID(QuixSparkMAX spark, int slot, double Kp, double Ki, double Kd) {
    this.pid = spark.controller.getPIDController();
    this.slot = slot;

    this.pid.setP(Kp, slot);
    this.pid.setI(Ki, slot);
    this.pid.setD(Kd, slot);
  }

  /** @param setpoint */
  public void setSetpointVelocity(double setpoint) {
    pid.setReference(setpoint, ControlType.kVelocity, slot, 0.0, ArbFFUnits.kVoltage);
  }

  /**
   * @param setpoint
   * @param feedforwardVolts
   */
  public void setSetpointVelocity(double setpoint, double feedforwardVolts) {
    pid.setReference(setpoint, ControlType.kVelocity, slot, feedforwardVolts, ArbFFUnits.kVoltage);
  }

  /**
   * Sets the proportional gain of the controller.
   *
   * @param Kp The proportional gain.
   */
  public void setP(double Kp) {
    pid.setP(Kp, slot);
  }

  /**
   * Sets the integral gain of the controller.
   *
   * @param Kp The integral gain.
   */
  public void setI(double Ki) {
    pid.setP(Ki, slot);
  }

  /**
   * Sets the derivative gain of the controller.
   *
   * @param Kp The derivative gain.
   */
  public void setD(double Kd) {
    pid.setP(Kd, slot);
  }

  /**
   * Returns proportional gain.
   *
   * @return P gain.
   */
  public double getP() {
    return pid.getP(slot);
  }

  /**
   * Returns integral gain.
   *
   * @return I gain.
   */
  public double getI() {
    return pid.getI(slot);
  }

  /**
   * Returns derivative gain.
   *
   * @return D gain.
   */
  public double getD() {
    return pid.getD(slot);
  }
}
