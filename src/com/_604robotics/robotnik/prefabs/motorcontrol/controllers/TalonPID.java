package com._604robotics.robotnik.prefabs.motorcontrol.controllers;

import com._604robotics.robotnik.prefabs.devices.FalconEncoder;
import com._604robotics.robotnik.prefabs.motorcontrol.QuixTalonFX;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class TalonPID extends MotorControllerPID{
  public TalonFX controller;
  private FalconEncoder encoder;

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
  public TalonPID(QuixTalonFX talon, FalconEncoder encoder, MotorControllerPIDConfig config) {
    super(talon, encoder, config);
    this.controller = talon.controller;
    this.encoder = encoder;

    this.controller.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    this.setConfig(config);
  }

  @Override
  public void setSetpointVelocity(double setpoint) {
    this.controller.set(
        TalonFXControlMode.Velocity, setpoint * (1.0 / 100.0), DemandType.ArbitraryFeedForward, 0.0);
  }

  @Override
  public void setSetpointVelocity(double setpoint, double feedforwardVolts) {
    this.controller.set(
        TalonFXControlMode.Velocity,
        (setpoint / encoder.getPositionConversionFactor()) * (1.0 / 100.0),
        DemandType.ArbitraryFeedForward,
        feedforwardVolts / 12.0);
  }

  @Override
  public void setSetpointPosition(double setpoint) {
    this.controller.set(TalonFXControlMode.Position, (setpoint / encoder.getPositionConversionFactor()), DemandType.ArbitraryFeedForward, 0.0);
  }

  @Override
  public void setSetpointPosition(double setpoint, double feedforwardVolts) {
    this.controller.set(TalonFXControlMode.Position, setpoint, DemandType.ArbitraryFeedForward, feedforwardVolts / 12.0);
  }

  @Override
  public void setConfig(MotorControllerPIDConfig config) {
    this.controller.config_kP(config.slot, config.Kp);
    this.controller.config_kI(config.slot, config.Ki);
    this.controller.config_kD(config.slot, config.Kd);
    this.controller.config_kF(config.slot, config.Kf);
  }
}
