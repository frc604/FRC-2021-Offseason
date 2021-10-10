
package com._604robotics.robotnik.prefabs.devices;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class QuixCANCoder implements BetterAbsoluteEncoder, Sendable{
    public CANCoder encoder;
    private String name;

    private double conversionFactor = 1.0;

    public QuixCANCoder(int port, String name) {
        this.encoder = new CANCoder(port);
        this.name = name;

        this.encoder.configFactoryDefault();
        this.encoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        this.encoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
    }

    @Override
    public boolean getInverted() {
        return encoder.configGetSensorDirection();
    }

    @Override
    public void setInverted(boolean inverted) {
        encoder.configSensorDirection(inverted);
    }

    @Override
    public void setdistancePerRotation(double distancePerRotation) {
        this.conversionFactor = distancePerRotation;        
    }

    @Override
    public double getPositionConversionFactor() {
      return conversionFactor;
    }
  
    @Override
    public double getVelocityConversionFactor() {
      return conversionFactor;
    }

    @Override
    public double getPosition() {    
        return encoder.getPosition() * conversionFactor;
    }

    @Override
    public double getAbsPosition() {
        return encoder.getAbsolutePosition();
    }

    @Override
    public double getVelocity() {
        return encoder.getVelocity() * conversionFactor;
    }

    @Override
    public void zero() {
        encoder.setPosition(0);
    }

    @Override
    public void zero(double value) {
        encoder.setPosition(value * conversionFactor);
    }

    @Override
    public void zeroToAbsPosition() {
        encoder.setPositionToAbsolute();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Gyro");
        builder.addDoubleProperty(name, this::getAbsPosition, null);
    }
}