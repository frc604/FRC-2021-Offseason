package com._604robotics.robotnik.prefabs.coordinators;

import static edu.wpi.first.wpilibj.util.ErrorMessages.requireNonNullParam;

import com._604robotics.robotnik.Coordinator;
import java.util.function.BooleanSupplier;

public class SleepUntilCoordinator extends Coordinator {

  private final BooleanSupplier condition;

  public SleepUntilCoordinator(BooleanSupplier condition) {
    this.condition = requireNonNullParam(condition, "condition", "SleepUntilCommand");
  }

  @Override
  protected boolean run() {
    return !condition.getAsBoolean();
  }
}
