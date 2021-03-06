package com._604robotics.robotnik.prefabs.motorcontrol.controllers;

import com._604robotics.robotnik.prefabs.devices.NEOEncoder;
import com._604robotics.robotnik.prefabs.motorcontrol.QuixSparkMAX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANPIDController.ArbFFUnits;
import com.revrobotics.ControlType;

public class SparkPID extends MotorControllerPID {
  public CANPIDController pid;

  /**
   * A wrapper representing a PID controller running on a REV Robotics SparkMAX.
   *
   * @param spark The QuixSparkMAX({@link
   *     com._604robotics.robotnik.prefabs.motorcontrol.QuixSparkMAX}) object that the controller is
   *     running on.
   */
  public SparkPID(QuixSparkMAX spark, NEOEncoder encoder, MotorControllerPIDConfig config) {
    super(spark, encoder, config);
    this.pid = spark.controller.getPIDController();

    this.setConfig(config);
  }

  @Override
  public void setSetpointVelocity(double setpoint) {
    pid.setReference(setpoint, ControlType.kVelocity, config.slot, 0.0, ArbFFUnits.kVoltage);
  }

  @Override
  public void setSetpointVelocity(double setpoint, double feedforwardVolts) {
    pid.setReference(setpoint, ControlType.kVelocity, config.slot, feedforwardVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void setSetpointPosition(double setpoint) {
    pid.setReference(setpoint, ControlType.kPosition, config.slot, 0.0, ArbFFUnits.kVoltage);
  }

  
  @Override
  public void setSetpointPosition(double setpoint, double feedforwardVolts) {
    pid.setReference(setpoint, ControlType.kPosition, config.slot, feedforwardVolts, ArbFFUnits.kVoltage);
  }

  @Override
  public void setConfig(MotorControllerPIDConfig config) {
    this.config = config;

    this.pid.setP(config.Kp);
    this.pid.setI(config.Ki);
    this.pid.setD(config.Kd);
    this.pid.setFF(config.Kf);
  }
}
