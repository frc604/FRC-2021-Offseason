package com._604robotics.robotnik.prefabs.devices;

public interface BetterAbsoluteEncoder {

public boolean getInverted();

public void setInverted(boolean inverted);

public void setdistancePerRotation(double distancePerRotation);

public double getPositionConversionFactor();

public double getVelocityConversionFactor();

public double getPosition();

public double getAbsPosition();

public double getVelocity();

public void zero();

public void zero(double value);

public void zeroToAbsPosition();

}
