package com._604robotics.robotnik.prefabs.devices;

import com.analog.adis16470.frc.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

public class ADIS16470 implements Sendable {
  private static ADIS16470 single_instance = null;
  private final ADIS16470_IMU imu;

  public static ADIS16470 getInstance() {
    if (single_instance == null) single_instance = new ADIS16470();

    return single_instance;
  }

  public ADIS16470() {
    imu = new ADIS16470_IMU();
    imu.configCalTime(ADIS16470_IMU.ADIS16470CalibrationTime._8s);
    imu.setYawAxis(ADIS16470_IMU.IMUAxis.kZ);

    SendableRegistry.addLW(this, "ADIS16470");
  }

  public void reset() {
    imu.reset();
  }

  public void calibrate() {
    imu.calibrate();
  }

  public void calibrate(ADIS16470_IMU.ADIS16470CalibrationTime time) {
    imu.configCalTime(time);
    imu.calibrate();
  }

  public double getAngle() {
    return -imu.getAngle();
  }

  public double getRate() {
    return -imu.getRate();
  }

  public double getYAccel() {
    return imu.getAccelInstantY();
  }

  public double getXAccel() {
    return imu.getAccelInstantX();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("Gyro");
    builder.addDoubleProperty("Value", this::getAngle, null);
  }
}
