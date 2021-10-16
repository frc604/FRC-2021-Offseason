package com._604robotics.robot2020.auto;

import java.util.ArrayList;
import java.util.List;

import com._604robotics.robot2020.modules.Swerve;
import com._604robotics.robotnik.prefabs.auto.QuikPlanSwerveReader;
import com._604robotics.robotnik.prefabs.auto.SwerveTrackerConstants;
import com._604robotics.robotnik.prefabs.swerve.QuixSwerveModuleState;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;

public class QuixPlanSwerveTrajectoryTracker extends QuixPlanSwerveTrackingCoordinator {
    private final SimpleMotorFeedforward feedforward;
    private final Swerve.Auto auto;

    public QuixPlanSwerveTrajectoryTracker(QuikPlanSwerveReader reader, Swerve swerve, SwerveTrackerConstants constants) {
        super(reader, swerve, constants);
        this.feedforward = constants.feedforward;

        this.auto = swerve.new Auto();
    }

    @Override
    public void useOutput(QuixSwerveModuleState[] moduleStates) {
        auto.moduleStates.set(moduleStates);

        double[] feedforwards = {};
        for (int i = 0; i < moduleStates.length; i++) {
            feedforwards[i] = feedforward.calculate(moduleStates[i].speedMetersPerSecond);
        }
        auto.feedforwards.set(feedforwards);

        auto.activate();
    }

    @Override
    public void stop() {
        auto.moduleStates.set(new QuixSwerveModuleState[4]);
        auto.feedforwards.set(new double[4]);

        auto.activate();
    }
}
