package com._604robotics.robotnik.prefabs.motorcontrol;

import com._604robotics.robotnik.Module;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class QuixTalonFX extends MotorController {

  public WPI_TalonFX controller;

  private String name;

  private Motor motor;

  public QuixTalonFX(int port, String name, Motor motor, Module module) {
    super(module);
    this.motor = motor;

    this.controller = new WPI_TalonFX(port);

    controller.configFactoryDefault();

    controller.enableVoltageCompensation(true);
    controller.configVoltageCompSaturation(12);

    controller.configNeutralDeadband(0.0);

    PowerMonitor.getInstance().addController(this, name);
  }

  public String getName() {
    return name;
  }

  public WPI_TalonFX getController() {
    return controller;
  }

  @Override
  public void set(double power) {
    controller.set(power);
  }

  @Override
  public void enableCurrentLimit(boolean enable) {
    super.isLimiting = enable;

    controller.configSupplyCurrentLimit(
        new SupplyCurrentLimitConfiguration(enable, getCurrentLimit(), 15, 0.5));
  }

  public void setVoltageCompSaturation(double volts, boolean enable) {
    controller.configVoltageCompSaturation(volts);
    controller.enableVoltageCompensation(enable);
  }

  public void follow(QuixTalonFX master, boolean inverted) {
    controller.follow(master.controller);
  }

  @Override
  public double getOutputCurrent() {
    return controller.getSupplyCurrent();
  }

  @Override
  public double get() {
    return controller.get();
  }

  @Override
  public void setInverted(boolean inverted) {
    controller.setInverted(inverted);
  }

  @Override
  public boolean getInverted() {
    return controller.getInverted();
  }

  @Override
  public void disable() {
    controller.disable();
  }

  @Override
  public void stopMotor() {
    controller.stopMotor();
  }

  @Override
  public double getOutputVoltage() {
    return controller.get() * getInputVoltage();
  }

  @Override
  public double getInputVoltage() {
    return controller.getBusVoltage();
  }
}
