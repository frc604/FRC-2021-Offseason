package com._604robotics.robotnik.prefabs.swerve;

import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.prefabs.devices.BetterAbsoluteEncoder;
import com._604robotics.robotnik.prefabs.devices.IntegratedEncoder;
import com._604robotics.robotnik.prefabs.motorcontrol.MotorController;
import com._604robotics.robotnik.prefabs.motorcontrol.controllers.MotorControllerPID;
import com._604robotics.robotnik.prefabs.motorcontrol.gearing.CalculableRatio;
import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
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
    protected SimpleMotorFeedforward driveFeedforward;
    protected CalculableRatio driveRatio;
    protected CalculableRatio steeringRatio;

    protected double angleOffset;
    protected double wheelDiameter;
    protected double maxDriveVelocity;

    private double lastAngle;


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
        SimpleMotorFeedforward driveFeedforward,
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
        this.driveFeedforward = driveFeedforward;
        this.driveRatio = driveRatio;
        this.steeringRatio = steeringRatio;
        this.angleOffset = angleOffset;
        this.wheelDiameter = wheelDiameter;
        this.maxDriveVelocity = maxDriveVelocity;
        
        this.driveMotor.setCurrentLimit(40);
        this.driveMotor.enableCurrentLimit(true);

        this.steeringMotor.setCurrentLimit(30);
        this.steeringMotor.enableCurrentLimit(true);
        
        this.driveEncoder.setdistancePerRotation(this.driveRatio.calculate(Math.PI * wheelDiameter));
        this.steeringEncoder.setdistancePerRotation(this.steeringRatio.calculate(360.0));

        zeroToAbsPosition();

        lastAngle = getState().angle.getDegrees();
    }

    // public void setDesiredStateClosedLoop(QuixSwerveModuleState desiredState) {
    //     desiredState = QuixSwerveModuleState.optimize(desiredState, getState().angle);

    //     drivePID.setSetpointVelocity(desiredState.speedMetersPerSecond);

    //     if (desiredState.speedMetersPerSecond >= 1e-6) {
    //         steeringPID.setSetpointPosition(desiredState.angle.getRadians());
    //     }
    // }

    // public void setDesiredStateClosedLoop(QuixSwerveModuleState desiredState, double feedforwardVolts) {
    //     desiredState = QuixSwerveModuleState.optimize(desiredState, getState().angle);

    //     drivePID.setSetpointVelocity(desiredState.speedMetersPerSecond, feedforwardVolts);

    //     if (desiredState.speedMetersPerSecond >= 1e-6) {
    //         steeringPID.setSetpointPosition(desiredState.angle.getRadians());
    //     }
    // }

    // public void setDesiredStateOpenLoop(QuixSwerveModuleState desiredState) {
    //     desiredState = QuixSwerveModuleState.optimize(desiredState, getState().angle);

    //     driveMotor.set(desiredState.speedMetersPerSecond / maxDriveVelocity);
    
    //     if (desiredState.speedMetersPerSecond >= 1e-3) {
    //         steeringPID.setSetpointPosition(desiredState.angle.getRadians());
    //     }
    // }

    public void setDesiredStateClosedLoop(QuixSwerveModuleState desiredState){
        desiredState = QuixSwerveModuleState.optimize(desiredState, getState().angle);

        drivePID.setSetpointVelocity(-desiredState.speedMetersPerSecond, driveFeedforward.calculate(desiredState.speedMetersPerSecond));

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= maxDriveVelocity * 0.01) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        steeringPID.setSetpointPosition(angle);
        lastAngle = angle;
    }


    public void setDesiredStateOpenLoop(QuixSwerveModuleState desiredState){
        desiredState = QuixSwerveModuleState.optimize(desiredState, getState().angle);

        double percentOutput = desiredState.speedMetersPerSecond / maxDriveVelocity;
        driveMotor.set(percentOutput);

        double angle = (Math.abs(desiredState.speedMetersPerSecond) <= maxDriveVelocity * 0.01) ? lastAngle : desiredState.angle.getDegrees(); //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        steeringPID.setSetpointPosition(angle);
        lastAngle = angle;
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
        steeringEncoder.zero((absSteeringEncoder.getAbsPosition() - angleOffset));
    }

    public double getAngle() {
        return this.steeringEncoder.getPosition();
    }
    
    public QuixSwerveModuleState getState(){
        double velocity = this.driveEncoder.getVelocity();
        Rotation2d angle = Rotation2d.fromDegrees(this.steeringEncoder.getPosition());
        return new QuixSwerveModuleState(velocity, angle);
    }
}
