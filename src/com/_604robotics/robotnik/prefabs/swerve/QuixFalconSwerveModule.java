package com._604robotics.robotnik.prefabs.swerve;

import com._604robotics.robotnik.Module;
import com._604robotics.robotnik.prefabs.devices.FalconEncoder;
import com._604robotics.robotnik.prefabs.devices.QuixCANCoder;
import com._604robotics.robotnik.prefabs.motorcontrol.Motor;
import com._604robotics.robotnik.prefabs.motorcontrol.QuixTalonFX;
import com._604robotics.robotnik.prefabs.motorcontrol.controllers.MotorControllerPIDConfig;
import com._604robotics.robotnik.prefabs.motorcontrol.controllers.TalonPID;
import com._604robotics.robotnik.prefabs.motorcontrol.gearing.CalculableRatio;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Translation2d;


public class QuixFalconSwerveModule extends QuixSwerveModule{

    public QuixFalconSwerveModule(
        String name,
        int id,
        Module module,
        Translation2d position,
        QuixTalonFX driveMotor,
        QuixTalonFX steeringMotor,
        FalconEncoder driveEncoder,
        FalconEncoder steeringEncoder,
        QuixCANCoder absSteeringEncoder,
        TalonPID drivePID,
        TalonPID steeringPID,
        SimpleMotorFeedforward driveFeedforward,
        CalculableRatio driveRatio,
        CalculableRatio steeringRatio,
        double angleOffset,
        double wheelDiameter,
        double maxDriveVelocity
    ) {
        super(name, id, module, position, driveMotor, steeringMotor, absSteeringEncoder, driveEncoder, steeringEncoder, drivePID, steeringPID, driveFeedforward, driveRatio, steeringRatio, angleOffset, wheelDiameter, maxDriveVelocity);

        driveMotor.controller.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        steeringMotor.controller.configIntegratedSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
    }


    public static QuixFalconSwerveModule createModule(
        String name,
        int id,
        Module module,
        Translation2d position,
        int driveMotorPort,
        int steeringMotorPort,
        int absSteeringEncoderPort,
        boolean invertDriveMotor,
        boolean invertSteeringMotor,
        MotorControllerPIDConfig drivePIDConfig,
        MotorControllerPIDConfig steeringPIDConfig,
        SimpleMotorFeedforward driveFeedforward,
        CalculableRatio driveRatio,
        CalculableRatio steeringRatio,
        double angleOffset,
        double wheelDiameter,
        double maxDriveVelocity
    ) {
        QuixTalonFX driveMotor = new QuixTalonFX(driveMotorPort, name + " Drive Motor", Motor.kFalcon500, module);
        QuixTalonFX steeringMotor = new QuixTalonFX(steeringMotorPort, name + " Steering Motor", Motor.kFalcon500, module);

        QuixCANCoder absSteeringEncoder = new QuixCANCoder(absSteeringEncoderPort, name + " Absolute Steering Encoder");

        driveMotor.setInverted(invertDriveMotor);
        steeringMotor.setInverted(invertSteeringMotor);

        driveMotor.controller.setNeutralMode(NeutralMode.Brake);
        steeringMotor.controller.setNeutralMode(NeutralMode.Coast);

        FalconEncoder driveEncoder = new FalconEncoder(driveMotor);
        FalconEncoder steeringEncoder = new FalconEncoder(steeringMotor);

        TalonPID drivePID = new TalonPID(driveMotor, driveEncoder, drivePIDConfig);
        TalonPID steeringPID = new TalonPID(steeringMotor, steeringEncoder, steeringPIDConfig);

        return new QuixFalconSwerveModule(name, id, module, position, driveMotor, steeringMotor, driveEncoder, steeringEncoder, absSteeringEncoder, drivePID, steeringPID, driveFeedforward, driveRatio, steeringRatio, angleOffset, wheelDiameter, maxDriveVelocity);
    }
}
