package com._604robotics.robotnik.prefabs.swerve;

import com._604robotics.robotnik.prefabs.devices.IntegratedEncoder;
import com._604robotics.robotnik.prefabs.motorcontrol.MotorController;
import com._604robotics.robotnik.prefabs.motorcontrol.controllers.MotorControllerPID;
import com._604robotics.robotnik.prefabs.motorcontrol.gearing.CalculableRatio;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public abstract class QuixSwerveModule {
    private String name;
    private int id;
    private MotorController driveMotor;
    private MotorController steeringMotor;
    private IntegratedEncoder driveEncoder;
    private IntegratedEncoder steeringEncoder;
    private MotorControllerPID drivePID;
    private MotorControllerPID steeringPID;
    private CalculableRatio driveRatio;
    private CalculableRatio steeringRatio;

    private double maxDriveVelocity;


    public QuixSwerveModule(
        String name,
        int id,
        MotorController driveMotor,
        MotorController steeringMotor,
        IntegratedEncoder driveEncoder,
        IntegratedEncoder steeringEncoder,
        MotorControllerPID drivePID,
        MotorControllerPID steeringPID,
        CalculableRatio driveRatio,
        CalculableRatio steeringRatio,
        double maxDriveVelocity
    ) {
        this.name = name;
        this.id = id;
        this.driveMotor = driveMotor;
        this.steeringMotor = steeringMotor;
        this.driveEncoder = driveEncoder;
        this.steeringEncoder = steeringEncoder;
        this.drivePID = drivePID;
        this.steeringPID = steeringPID;
        this.driveRatio = driveRatio;
        this.steeringRatio = steeringRatio;
        this.maxDriveVelocity = maxDriveVelocity;
    }

    public void setDesiredStateClosedLoop(QuixSwerveModuleState desiredState) {
        desiredState = QuixSwerveModuleState.optimize(desiredState, getState().angle);

        drivePID.setSetpointVelocity(desiredState.speedMetersPerSecond);
        steeringPID.setSetpointPosition(desiredState.angle.getRadians());
    }

    public void setDesiredStateOpenLoop(QuixSwerveModuleState desiredState) {
        desiredState = QuixSwerveModuleState.optimize(desiredState, getState().angle);

        driveMotor.set(desiredState.speedMetersPerSecond / maxDriveVelocity);
        steeringPID.setSetpointPosition(desiredState.angle.getRadians());
    }
    
    public QuixSwerveModuleState getState(){
        double velocity = this.driveEncoder.getVelocity();
        Rotation2d angle = new Rotation2d(this.steeringEncoder.getPosition());
        return new QuixSwerveModuleState(velocity, angle);
    }
}
