package com._604robotics.robotnik.prefabs.motorcontrol;

import com._604robotics.robotnik.Module;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.RobotController;

public class QuixVictorSPX extends MotorController {

  public WPI_VictorSPX controller;

  private String name;

  private int PDPport;

  public QuixVictorSPX(int port, String name, int PDPport, Module module) {
    super(module);
    controller = new WPI_VictorSPX(port);
    controller.configVoltageCompSaturation(12);
    controller.enableVoltageCompensation(true);
    this.PDPport = PDPport;

    PowerMonitor.getInstance().addController(this, name);
  }

  public QuixVictorSPX(int port, String name, Module module) {
    super(module);
    controller = new WPI_VictorSPX(port);
    controller.configVoltageCompSaturation(12);
    controller.enableVoltageCompensation(true);

    PowerMonitor.getInstance().addController(this, name);

    PDPport = -1;
  }

  public String getName() {
    return name;
  }

  @Override
  public void set(double power) {
    controller.set(power);
  }

  public void setVoltageCompSaturation(double volts, boolean enable) {
    controller.configVoltageCompSaturation(volts);
    controller.enableVoltageCompensation(enable);
  }

  @Override
  public double getOutputCurrent() {
    if (PDPport >= 0) {
      return PowerMonitor.getInstance().panel.getCurrent(PDPport);
    } else {
      setCurrentLimit(-1);
      throw new UnsupportedOperationException(
          "If no PDP port is provided the current output cannot be tracked!");
    }
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
    set(0.0);
  }

  @Override
  public double getOutputVoltage() {
    return controller.get() * getInputVoltage();
  }

  @Override
  public double getInputVoltage() {
    return RobotController.getBatteryVoltage();
  }

  @Override
  public int getCurrentLimit() {
    return -1;
  }

  @Override
  public boolean isCurrentLimiting() {
    return false;
  }
}
