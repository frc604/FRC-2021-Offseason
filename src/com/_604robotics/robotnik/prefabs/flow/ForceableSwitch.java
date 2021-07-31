package com._604robotics.robotnik.prefabs.flow;

import java.util.function.BooleanSupplier;

public class ForceableSwitch {
  private BooleanSupplier on;
  private BooleanSupplier off;
  private Runnable onAction;
  private Runnable offAction;

  private boolean forceableSwitch = false;

  private boolean force = false;

  private boolean isForcing = false;

  public ForceableSwitch(
      BooleanSupplier on,
      BooleanSupplier off,
      Runnable onAction,
      Runnable offAction,
      boolean startOn,
      boolean startForceOn) {
    this.on = on;
    this.off = off;
    this.onAction = onAction;
    this.offAction = offAction;

    if (startOn) {
      forceableSwitch = true;
    }

    if (startForceOn) {
      force = true;
    }
  }

  public void force(boolean on) {
    force = on;
    update();
  }

  public void toggleForcing(boolean forcing) {
    isForcing = forcing;
  }

  public void update() {
    if (!isForcing) {
      if (on.getAsBoolean() && !forceableSwitch) {
        forceableSwitch = true;
      } else if (off.getAsBoolean() && forceableSwitch) {
        forceableSwitch = false;
      }
    }
    if (isForcing ? force : forceableSwitch) {
      onAction.run();
    } else {
      offAction.run();
    }
  }
}
