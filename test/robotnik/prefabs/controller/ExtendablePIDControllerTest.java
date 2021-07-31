/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
/* This is 604's custom version to test the re-written multithreaded          */
/* controllers                                                                */
/*----------------------------------------------------------------------------*/
package robotnik.prefabs.controller;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com._604robotics.robotnik.prefabs.controller.ExtendablePIDController;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

class ExtendablePIDControllerTest {
  private ExtendablePIDController controller;
  private double measurement = 0.0;
  private double output = 0.0;

  @BeforeEach
  void setUp() {
    controller =
        new ExtendablePIDController(0, 0, 0, () -> measurement, (output) -> this.output = output);
    controller.getNotifier().stop();
  }

  @Test
  void continuousInputTest() {
    controller.setP(1);
    controller.enableContinuousInput(-180, 180);

    controller.setSetpoint(179);
    measurement = -179;
    controller.setEnabled(true);

    waitForCalculate(1);

    assertTrue(output < 0.0);
    controller.setEnabled(false);
  }

  @Test
  void proportionalGainOutputTest() {
    controller.setP(4);

    controller.setSetpoint(0);
    measurement = 0.025;
    controller.setEnabled(true);

    waitForCalculate(1);

    assertEquals(-0.1, output, 1e-5);
    controller.setEnabled(false);
  }

  @Test
  void integralGainOutputTest() {
    controller.setI(4);

    controller.setSetpoint(0);
    measurement = 0.025;
    controller.setEnabled(true);

    waitForCalculate(5);

    // Larger delta due to a time issue with unit tests.
    assertEquals(-0.5 * controller.getPeriod(), output, 0.002);
    controller.setEnabled(false);
  }

  @Test
  void derivativeGainOutputTest() {
    controller.setD(4);

    controller.setEnabled(true);

    controller.setSetpoint(0);
    measurement = 0;

    waitForCalculate(1);

    controller.setSetpoint(0);
    measurement = 0.0025;

    waitForCalculate(1);

    assertEquals(-0.01 / controller.getPeriod(), output, 1e-5);
    controller.setEnabled(false);
  }

  @Test
  void outputClampTest() {
    controller.setPID(1, 0, 0);

    controller.setOutputRange(-12, 12);

    controller.setEnabled(true);

    controller.setSetpoint(20);
    measurement = 1;

    waitForCalculate(1);

    assertEquals(12, output, 1e-5);
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
