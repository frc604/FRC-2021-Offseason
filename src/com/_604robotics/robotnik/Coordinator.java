package com._604robotics.robotnik;

import com._604robotics.robotnik.prefabs.coordinators.ParallelRaceCoordinator;
import com._604robotics.robotnik.prefabs.coordinators.SleepCoordinator;
import com._604robotics.robotnik.prefabs.coordinators.SleepUntilCoordinator;
import java.util.function.BooleanSupplier;

public abstract class Coordinator {
  private boolean running;

  public boolean isRunning() {
    return running;
  }

  public void start() {
    if (!running) {
      running = true;
      begin();
    }
  }

  public boolean execute() {
    if (running) {
      running = run();
      if (!running) {
        end();
      }
    }
    return running;
  }

  public void stop() {
    if (running) {
      running = false;
      end();
    }
  }

  public ParallelRaceCoordinator withTimeout(double seconds) {
    return new ParallelRaceCoordinator(this.toString(), this, new SleepCoordinator(seconds));
  }

  public ParallelRaceCoordinator withInterrupt(BooleanSupplier condition) {
    return new ParallelRaceCoordinator(this.toString(), this, new SleepUntilCoordinator(condition));
  }

  protected void begin() {}
  /**
   * Returns FALSE when FINISHED. FALSE. FINISHED. BOTH START WITH 'F'.
   *
   * @return boolean representing whether this Coordinator is running. IS RUNNING.
   */
  protected boolean run() {
    return true;
  }

  protected void end() {}
}
