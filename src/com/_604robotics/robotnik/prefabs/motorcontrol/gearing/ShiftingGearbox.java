package com._604robotics.robotnik.prefabs.motorcontrol.gearing;

import com._604robotics.robotnik.prefabs.modules.Shifter;

public class ShiftingGearbox implements CalculableRatio {
  private GearRatio stage1;
  private GearRatio stage2;
  private GearRatio lowGear;
  private GearRatio highGear;

  private Shifter shifter;

  public ShiftingGearbox(
      GearRatio stage1, GearRatio stage2, GearRatio lowGear, GearRatio highGear, Shifter shifter) {
    this.stage1 = stage1;
    this.stage2 = stage2;
    this.lowGear = lowGear;
    this.highGear = highGear;

    this.shifter = shifter;
  }

  @Override
  public double calculate(double input) {
    if (shifter.inHighGear.get()) {
      return highGear.calculate(stage2.calculate(stage1.calculate(input)));
    } else {
      return lowGear.calculate(stage2.calculate(stage1.calculate(input)));
    }
  }
}
