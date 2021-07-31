package com._604robotics.robotnik.prefabs.motorcontrol.controllers;

import com._604robotics.robotnik.prefabs.devices.FalconEncoder;
import com._604robotics.robotnik.prefabs.motorcontrol.QuixTalonFX;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class TalonPID {
  public TalonFX controller;

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
  public TalonPID(QuixTalonFX talon, FalconEncoder encoder, double Kp, double Ki, double Kd) {
    this.controller = talon.controller;

    this.controller.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    this.controller.configSelectedFeedbackCoefficient(encoder.getPositonConversionFactor());
    this.controller.config_kP(slot, Kp);
    this.controller.config_kI(slot, Ki);
    this.controller.config_kD(slot, Kd);
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
  public TalonPID(
      QuixTalonFX talon, FalconEncoder encoder, int slot, double Kp, double Ki, double Kd) {
    this.controller = talon.controller;
    this.slot = slot;

    this.controller.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    this.controller.configSelectedFeedbackCoefficient(encoder.getPositonConversionFactor());
    this.controller.config_kP(slot, Kp);
    this.controller.config_kI(slot, Ki);
    this.controller.config_kD(slot, Kd);
  }

  /** @param setpoint */
  public void setSetpointVelocity(double setpoint) {
    controller.set(
        TalonFXControlMode.Velocity, setpoint * (1.0 / 10.0), DemandType.ArbitraryFeedForward, 0.0);
  }
  /**
   * @param setpoint
   * @param feedforwardVolts
   */
  public void setSetpointVelocity(double setpoint, double feedforwardVolts) {
    controller.set(
        TalonFXControlMode.Velocity,
        setpoint * (1.0 / 10.0),
        DemandType.ArbitraryFeedForward,
        feedforwardVolts / 12.0);
  }

  /**
   * Sets the proportional gain of the controller.
   *
   * @param Kp The proportional gain.
   */
  public void setP(double Kp) {
    this.controller.config_kP(slot, Kp);
  }

  /**
   * Sets the integral gain of the controller.
   *
   * @param Kp The integral gain.
   */
  public void setI(double Ki) {
    this.controller.config_kI(slot, Ki);
  }

  /**
   * Sets the derivative gain of the controller.
   *
   * @param Kp The derivative gain.
   */
  public void setD(double Kd) {
    this.controller.config_kD(slot, Kd);
  }
}
