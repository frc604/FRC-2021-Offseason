package com._604robotics.robotnik.prefabs.swerve;

import com._604robotics.robotnik.prefabs.devices.FalconEncoder;
import com._604robotics.robotnik.prefabs.devices.IntegratedEncoder;
import com._604robotics.robotnik.prefabs.motorcontrol.MotorController;
import com._604robotics.robotnik.prefabs.motorcontrol.QuixTalonFX;
import com._604robotics.robotnik.prefabs.motorcontrol.controllers.TalonPID;
import com._604robotics.robotnik.prefabs.motorcontrol.gearing.CalculableRatio;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;


public class QuixFalconSwerveModule extends QuixSwerveModule{
    public QuixFalconSwerveModule(
        String name,
        int id,
        QuixTalonFX driveMotor,
        QuixTalonFX steeringMotor,
        FalconEncoder driveEncoder,
        FalconEncoder steeringEncoder,
        TalonPID drivePID,
        TalonPID steeringPID,
        CalculableRatio driveRatio,
        CalculableRatio steeringRatio,
        double maxDriveVelocity
    ) {
        super(name, id, driveMotor, steeringMotor, driveEncoder, steeringEncoder, drivePID, steeringPID, driveRatio, steeringRatio, maxDriveVelocity);

        driveMotor.controller.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        steeringMotor.controller.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    }
}
