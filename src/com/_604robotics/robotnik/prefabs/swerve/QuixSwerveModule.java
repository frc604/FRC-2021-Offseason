package com._604robotics.robotnik.prefabs.swerve;

import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.prefabs.devices.BetterAbsoluteEncoder;
import com._604robotics.robotnik.prefabs.devices.IntegratedEncoder;
import com._604robotics.robotnik.prefabs.motorcontrol.MotorController;
import com._604robotics.robotnik.prefabs.motorcontrol.controllers.MotorControllerPID;
import com._604robotics.robotnik.prefabs.motorcontrol.gearing.CalculableRatio;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public abstract class QuixSwerveModule {
    protected String name;
    protected int id;
    protected Module module;
    protected Translation2d position;
    protected MotorController driveMotor;
    protected MotorController steeringMotor;
    protected BetterAbsoluteEncoder absSteeringEncoder;
    protected IntegratedEncoder driveEncoder;
    protected IntegratedEncoder steeringEncoder;
    protected MotorControllerPID drivePID;
    protected MotorControllerPID steeringPID;
    protected CalculableRatio driveRatio;
    protected CalculableRatio steeringRatio;

    protected double angleOffset;
    protected double wheelDiameter;
    protected double maxDriveVelocity;


    public QuixSwerveModule(
        String name,
        int id,
        Module module,
        Translation2d position,
        MotorController driveMotor,
        MotorController steeringMotor,
        BetterAbsoluteEncoder absSteeringEncoder,
        IntegratedEncoder driveEncoder,
        IntegratedEncoder steeringEncoder,
        MotorControllerPID drivePID,
        MotorControllerPID steeringPID,
        CalculableRatio driveRatio,
        CalculableRatio steeringRatio,
        double angleOffset,
        double wheelDiameter,
        double maxDriveVelocity
    ) {
        this.name = name;
        this.id = id;
        this.module = module;
        this.position = position;
        this.driveMotor = driveMotor;
        this.steeringMotor = steeringMotor;
        this.absSteeringEncoder = absSteeringEncoder;
        this.driveEncoder = driveEncoder;
        this.steeringEncoder = steeringEncoder;
        this.drivePID = drivePID;
        this.steeringPID = steeringPID;
        this.driveRatio = driveRatio;
        this.steeringRatio = steeringRatio;
        this.angleOffset = angleOffset;
        this.wheelDiameter = wheelDiameter;
        this.maxDriveVelocity = maxDriveVelocity;
        
        this.driveEncoder.setdistancePerRotation(this.driveRatio.calculate(wheelDiameter));
        this.steeringEncoder.setdistancePerRotation(this.steeringRatio.calculate(2.0 * Math.PI));

        zeroToAbsPosition();
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

    public int getID() {
        return id;
    }

    public Translation2d getPosition() {
        return position;
    }

    public double getAbsEncoderAngle() {
        return absSteeringEncoder.getAbsPosition();
    }

    public void zeroToAbsPosition() {
        steeringEncoder.zero((absSteeringEncoder.getAbsPosition() - angleOffset) * (Math.PI / 180.0));
    }

    public double getAngle() {
        return this.steeringEncoder.getPosition() * (180.0 / Math.PI);
    }
    
    public QuixSwerveModuleState getState(){
        double velocity = this.driveEncoder.getVelocity();
        Rotation2d angle = new Rotation2d(this.steeringEncoder.getPosition());
        return new QuixSwerveModuleState(velocity, angle);
    }
}
