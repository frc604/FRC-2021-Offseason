package com._604robotics.robotnik;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import java.util.Collection;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.Map;

@SuppressWarnings({"rawtypes", "unchecked", "unused"})
public class DashboardManager {
  private static DashboardManager single_instance = null;

  private Robot robot;

  private HashMap<SimpleWidget, Input> inputs = new HashMap<>();

  public static DashboardManager getInstance() {
    if (single_instance == null) single_instance = new DashboardManager();

    return single_instance;
  }

  public DashboardManager() {}

  /**
   * Iterates through the modules of a robotnik Robot object({@link
   * com._604robotics.robotnik.Robot}) and registers all the inputs and outputs to a modules
   * respective shuffleboard tab. This allows for easy tracking and debugging of all the states of a
   * module.
   *
   * @param robot The robotnik Robot object to register.
   */
  public void registerRobot(Robot robot) {
    this.robot = robot;
    for (Module module : robot.getModules()) {

      if (!(module.getOutputs().size() == 0) || !(module.getInputs().size() == 0)) {
        ShuffleboardTab tab = Shuffleboard.getTab(module.getName());

        for (OutputProxy output : module.getOutputs()) {
          if (output.get() instanceof Boolean) {
            tab.addBoolean(output.getName(), () -> (Boolean) output.get());
          } else if (output.get() instanceof Double) {
            tab.addNumber(output.getName(), () -> (Double) output.get());
          } else if (output.get() instanceof Integer) {
            tab.addNumber(output.getName(), () -> (Integer) output.get());
          } else {
            tab.addString(output.getName(), () -> output.get().toString());
          }
        }

        for (Input input : module.getInputs()) {
          inputs.put(tab.add(input.getName(), input.get()), input);
        }
      }
    }
  }

  public void update() {
    for (Map.Entry<SimpleWidget, Input> input : inputs.entrySet()) {
      input.getValue().set(input.getKey().getEntry().getValue());
    }
  }

  /**
   * Registers a dashboard enum output chooser allowing a value from the dashboard is outputted to
   * the robot code. This is most commonly used to make drive mode or autonomous mode selectors.
   *
   * @param name The name of the enum output to register.
   * @param defaultValue The default value of the enum chooser.
   * @param klass The enum class for the chooser.
   * @param module The parent module of which to register the chooser.
   * @return Output<E> that is supplied with the selected option of the chooser.
   */
  public <E extends Enum<E>> Output<E> registerEnumOutput(
      String name, E defaultValue, Class<E> klass, Module module) {
    SendableChooser<E> chooser = new SendableChooser<>();

    for (E option : EnumSet.allOf(klass)) {
      if (option == defaultValue) {
        chooser.setDefaultOption(option.toString(), option);
      } else {
        chooser.addOption(option.toString(), option);
      }
    }

    var gonna = module;
    var it = chooser;

    String send = name + " Selector";

    /* registerSendable(module, name, chooser); */

    just(gonna, send, it);

    return module.addOutput(name, chooser::getSelected);
  }

  public <V> Output<V> registerCollection(
      String name, V defaultValue, Collection<V> collection, Module module) {
    SendableChooser<V> chooser = new SendableChooser<>();

    for (V option : collection) {
      if (option == defaultValue) {
        chooser.setDefaultOption(option.toString(), option);
      } else {
        chooser.addOption(option.toString(), option);
      }
    }

    var gonna = module;
    var it = chooser;

    String send = name + " Selector";

    /* registerSendable(module, name, chooser); */

    just(gonna, send, it);

    return module.addOutput(name, chooser::getSelected);
  }

  /**
   * Registers a dashboard enum input chooser allowing a enum from the robot code to be outputted to
   * the dashboard. This is can be used to show a measured color out of a list of different colors.
   *
   * @param name The name of the enum output to register.
   * @param initialValue The initial selected value to display onto the dashboard.
   * @param module The parent module of which to register the enum input.
   */
  public <E extends Enum<E>> void registerEnumInput(String name, E initialValue, Module module) {
    final Input<E> input = module.addInput(name, initialValue);
    Shuffleboard.getTab(module.getName()).addString(name, () -> input.get().toString());
  }

  /**
   * Registers a Sendable object({@link edu.wpi.first.wpilibj.Sendable}) to the dashboard.
   *
   * @param module The parent module of which to register the enum input.
   * @param name The name of the Sendable module to display on the dashboard.
   * @param sendable The sendable object to register to the dashboard.
   */
  public void registerSendable(Module module, String name, Sendable sendable) {
    Shuffleboard.getTab(module.getName()).add(name, sendable);
  }

  private void just(Module module, String name, Sendable sendable) {
    registerSendable(module, name, sendable);
  }
}
