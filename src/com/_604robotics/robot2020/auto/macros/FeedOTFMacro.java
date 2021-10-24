package com._604robotics.robot2020.auto.macros;

import java.util.function.BooleanSupplier;

import com._604robotics.robot2020.modules.AntiJamRoller;
import com._604robotics.robot2020.modules.Intake;
import com._604robotics.robot2020.modules.IntakeDeploy;
import com._604robotics.robot2020.modules.Revolver;
import com._604robotics.robot2020.modules.Tower;
import com._604robotics.robotnik.Coordinator;


public class FeedOTFMacro extends Coordinator {
  private Revolver.Empty revolverEmpty;
  private Tower.Empty towerEmpty;
  private AntiJamRoller.AntiJam roller;

  private Intake.Suck intakeSuck;
  private IntakeDeploy.Deploy deploy;

  private BooleanSupplier doShoot;

  public FeedOTFMacro(
    Intake intake,
    IntakeDeploy intakeDeploy,
    Revolver revolver,
    Tower tower,
    AntiJamRoller antiJam,
    BooleanSupplier doShoot) {
    this.revolverEmpty = revolver.empty;
    this.towerEmpty = tower.empty;
    this.roller = antiJam.antiJam;
    this.intakeSuck = intake.suck;
    this.deploy = intakeDeploy.deploy;
    this.doShoot = doShoot;
  }

  @Override
  public boolean run() {
    if (doShoot.getAsBoolean()) {
      revolverEmpty.activate();
      towerEmpty.activate();
      roller.activate();
      deploy.activate();
      intakeSuck.activate();
    } else {
      intakeSuck.activate();
      deploy.activate();
      revolverEmpty.activate();
      roller.activate();
    }
    return true;
  }
}
