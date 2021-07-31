package com._604robotics.robot2020.modules;

import com._604robotics.robotnik.Action;
import com._604robotics.robotnik.Module;

public class Climber extends Module {
  // private QuixSparkMAX climbMotor;

  public Climber() {
    super(Climber.class);

    // climbMotor = new QuixSparkMAX(Ports.CLIMBER_MOTOR, "Climber Motor", Motor.kNEO, this);

    setDefaultAction(retract);
  }

  public class Idle extends Action {

    public Idle() {
      super(Climber.this, Idle.class);
    }

    @Override
    public void begin() {
      // climbMotor.set(0.0);
    }
  }

  public class Retract extends Action {

    public Retract() {
      super(Climber.this, Retract.class);
    }

    @Override
    public void begin() {
      // climbMotor.set(Calibration.Climber.RETRACT_SPEED);
    }
  }

  public class Climb extends Action {

    public Climb() {
      super(Climber.this, Climb.class);
    }

    @Override
    public void run() {
      // climbMotor.set(Calibration.Climber.CLIMB_SPEED);
    }
  }

  public Action idle = new Idle();
  public Action climb = new Climb();
  public Action retract = new Retract();
}
