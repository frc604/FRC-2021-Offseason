package com._604robotics.robotnik.prefabs.devices;

import com._604robotics.robotnik.prefabs.motorcontrol.QuixTalonFX;
import com._604robotics.robotnik.prefabs.motorcontrol.gearing.CalculableRatio;
import com.ctre.phoenix.motorcontrol.TalonFXSensorCollection;

public class FalconEncoder implements Encoder {
  private final TalonFXSensorCollection sensors;

  private CalculableRatio ratio;

  private double conversionFactor = 1.0;
  private boolean inverted = false;

  public FalconEncoder(QuixTalonFX talon) {
    sensors = talon.controller.getSensorCollection();
    ratio = null;
  }

  public FalconEncoder(QuixTalonFX talon, CalculableRatio ratio) {
    sensors = talon.controller.getSensorCollection();
    this.ratio = ratio;
    conversionFactor = ratio.calculate(1.0);
  }

  public boolean getInverted() {
    return inverted;
  }

  public void setInverted(boolean inverted) {
    this.inverted = inverted;
  }

  @Override
  public double getValue() {
    return getPosition();
  }

  public void setdistancePerRotation(double distancePerRotation) {
    if (ratio == null) {
      conversionFactor = distancePerRotation * (1.0 / 2048.0);
    } else {
      conversionFactor = ratio.calculate(distancePerRotation * (1.0 / 2048.0));
    }
  }

  public double getPositonConversionFactor() {
    return conversionFactor;
  }

  public double getVelocityConversionFactor() {
    return conversionFactor * (1000.0 / 1.0);
  }

  public double getPosition() {
    return getPos();
  }

  public double getVelocity() {
    return getVel();
  }

  public void zero() {
    sensors.setIntegratedSensorPosition(0.0, 0);
  }

  public void zero(double value) {
    sensors.setIntegratedSensorPosition(value * conversionFactor, 0);
  }

  private double getPos() {
    double factor;

    if (inverted) {
      factor = -1.0;
    } else {
      factor = 1.0;
    }

    return sensors.getIntegratedSensorPosition() * factor * conversionFactor;
  }

  private double getVel() {
    double factor;

    if (inverted) {
      factor = -1.0;
    } else {
      factor = 1.0;
    }

    return sensors.getIntegratedSensorVelocity() * factor * conversionFactor * (1000.0 / 100.0);
  }
}
