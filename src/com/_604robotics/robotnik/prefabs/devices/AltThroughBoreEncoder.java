package com._604robotics.robotnik.prefabs.devices;

import com._604robotics.robotnik.prefabs.motorcontrol.QuixSparkMAX;
import com._604robotics.robotnik.prefabs.motorcontrol.gearing.CalculableRatio;
import com.revrobotics.AlternateEncoderType;
import com.revrobotics.CANEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AltThroughBoreEncoder implements Encoder {
  private final CANEncoder encoder;

  private CalculableRatio ratio;

  private boolean inverted = false;

  public AltThroughBoreEncoder(QuixSparkMAX spark) {
    encoder = spark.controller.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192);
    ratio = null;
  }

  public AltThroughBoreEncoder(QuixSparkMAX spark, CalculableRatio ratio) {
    encoder = spark.controller.getAlternateEncoder(AlternateEncoderType.kQuadrature, 8192);
    this.ratio = ratio;
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
      encoder.setPositionConversionFactor(distancePerRotation);
      encoder.setVelocityConversionFactor(distancePerRotation * (1.0 / 60.0));
    } else {
      encoder.setPositionConversionFactor(ratio.calculate(distancePerRotation));
      SmartDashboard.putNumber("REUDCC", ratio.calculate(1.0));
      System.out.println(
          "Soarks are dumb " + (ratio.calculate(distancePerRotation) * (1.0 / 60.0)));
      encoder.setVelocityConversionFactor(ratio.calculate(distancePerRotation) * (1.0 / 60.0));
    }
  }

  public double getdistancePerRotation() {
    return encoder.getPositionConversionFactor();
  }

  public double getPosition() {
    return getPos();
  }

  public double getVelocity() {
    return getVel();
  }

  public void zero() {
    encoder.setPosition(0.0);
  }

  public void zero(double value) {
    encoder.setPosition(value);
  }

  private double getPos() {
    int factor;

    if (inverted) {
      factor = -1;
    } else {
      factor = 1;
    }

    return encoder.getPosition() * factor;
  }

  private double getVel() {
    int factor;

    if (inverted) {
      factor = -1;
    } else {
      factor = 1;
    }

    return encoder.getVelocity() * factor;
  }
}
