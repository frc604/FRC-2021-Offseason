package com._604robotics.robotnik.prefabs.coordinators;

import com._604robotics.robotnik.Coordinator;
import com._604robotics.robotnik.Logger;
import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

public class ParallelRaceCoordinator extends Coordinator {
  private final Logger logger;

  private final Set<Coordinator> coordinators = new HashSet<>();
  private boolean finished = true;

  public ParallelRaceCoordinator(String name) {
    logger = new Logger(ParallelRaceCoordinator.class, name);
  }

  public ParallelRaceCoordinator(Class<?> klass) {
    this(klass.getSimpleName());
  }

  public ParallelRaceCoordinator(String name, Coordinator... coordinators) {
    logger = new Logger(ParallelCoordinator.class, name);

    addCoordinators(coordinators);
  }

  public ParallelRaceCoordinator(Class<?> klass, Coordinator... coordinators) {
    this(klass.getSimpleName());

    addCoordinators(coordinators);
  }

  public void addCoordinators(Coordinator... coordinators) {
    this.coordinators.addAll(Arrays.asList(coordinators));
  }

  @Override
  public void begin() {
    logger.info("Begin");
    finished = false;
    for (Coordinator c : this.coordinators) {
      logger.info("Starting " + c.toString());
      c.start();
    }
  }

  @Override
  public boolean run() {
    for (Coordinator c : this.coordinators) {
      c.execute();
      if (!c.isRunning()) {
        finished = true;
      }
    }

    return !finished;
  }

  @Override
  public void end() {
    for (Coordinator c : this.coordinators) {
      c.stop();
      logger.info("Stopped " + c.toString());
    }
    logger.info("End");
  }
}
