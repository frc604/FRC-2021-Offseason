/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* This is 604's custom version to test the re-written multithreaded          */
/* controllers                                                                */
/*----------------------------------------------------------------------------*/

package robotnik.prefabs.controller;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com._604robotics.robotnik.prefabs.controller.ExtendablePIDController;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ExtendablePIDControllerToleranceTest {
  private static final double kSetpoint = 50.0;
  private static final double kTolerance = 10.0;
  private static final double kRange = 200;

  private ExtendablePIDController controller;

  private double measurement = 0.0;

  @SuppressWarnings("unused")
  private double output = 0.0;

  @BeforeEach
  void setUp() {
    controller =
        new ExtendablePIDController(0, 0, 0, () -> measurement, (output) -> this.output = output);
    controller.getNotifier().stop();
  }

  @Test
  void initialToleranceTest() {
    controller.setPID(0.05, 0.0, 0.0);
    controller.enableContinuousInput(-kRange / 2, kRange / 2);

    assertTrue(controller.atSetpoint());
  }

  @Test
  void absoluteToleranceTest() {
    controller.setPID(0.05, 0.0, 0.0);
    controller.enableContinuousInput(-kRange / 2, kRange / 2);

    controller.setTolerance(kTolerance);
    controller.setSetpoint(kSetpoint);

    measurement = 0.025;
    controller.setEnabled(true);

    waitForCalculate(1);

    assertFalse(
        controller.atSetpoint(),
        "Error was in tolerance when it should not have been. Error was "
            + controller.getPositionError());

    controller.setEnabled(false);

    measurement = kSetpoint + kTolerance / 2;
    controller.setEnabled(true);

    waitForCalculate(1);

    assertTrue(
        controller.atSetpoint(),
        "Error was not in tolerance when it should have been. Error was "
            + controller.getPositionError());

    controller.setEnabled(false);

    measurement = kSetpoint + 10 * kTolerance;
    controller.setEnabled(true);

    waitForCalculate(1);

    assertFalse(
        controller.atSetpoint(),
        "Error was in tolerance when it should not have been. Error was "
            + controller.getPositionError());
    controller.setEnabled(false);
  }

  private void waitForCalculate(int periods) {
    long period = 25;
    try {
      for (int i = 0; i < periods; ++i) {
        controller.getNotifier().startSingle(0);
        Thread.sleep(period);
      }
    } catch (InterruptedException e) {
      e.printStackTrace();
    }
  }
}
