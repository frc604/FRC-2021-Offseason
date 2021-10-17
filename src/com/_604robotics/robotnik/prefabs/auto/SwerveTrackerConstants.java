package com._604robotics.robotnik.prefabs.auto;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class SwerveTrackerConstants {
    public PIDController xController;
    public PIDController yController;
    public PIDController thetaController;
    public double maxSpeed;
    public double maxAcceleration;

    public SwerveTrackerConstants(
        PIDController xController,
        PIDController yController,
        PIDController thetaController,
        double maxSpeed,
        double maxAcceleration) {
    this.xController = xController;
    this.yController = yController;
    this.thetaController = thetaController;
    this.maxSpeed = maxSpeed;
    this.maxAcceleration = maxAcceleration;
    }
}
