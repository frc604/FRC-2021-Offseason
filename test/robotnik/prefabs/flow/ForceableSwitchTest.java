package robotnik.prefabs.flow;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com._604robotics.robotnik.prefabs.flow.ForceableSwitch;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class ForceableSwitchTest {
  private boolean on = false;
  private boolean off = false;

  private boolean isOn = false;

  private ForceableSwitch forceableSwitch;

  @BeforeEach
  void setUp() {
    Runnable onRunnable = () -> isOn = true;

    Runnable offRunnable = () -> isOn = false;

    forceableSwitch =
        new ForceableSwitch(() -> on, () -> off, onRunnable, offRunnable, false, false);
  }

  @Test
  void toggleOn() {
    off = true;

    forceableSwitch.update();

    off = false;
    on = true;

    forceableSwitch.update();

    assertTrue(isOn);
  }

  @Test
  void toggleOff() {
    on = true;

    forceableSwitch.update();

    on = false;
    off = true;

    forceableSwitch.update();

    assertFalse(isOn);
  }

  @Test
  void forceOn() {
    on = true;

    forceableSwitch.update();

    on = false;
    off = true;

    forceableSwitch.update();

    forceableSwitch.toggleForcing(true);

    forceableSwitch.force(true);

    assertTrue(isOn);
  }

  @Test
  void forceOff() {
    off = true;

    forceableSwitch.update();

    off = false;
    on = true;

    forceableSwitch.update();

    forceableSwitch.toggleForcing(true);

    forceableSwitch.force(false);

    assertFalse(isOn);
  }

  @Test
  void forcingToggle() {
    off = true;

    forceableSwitch.update();

    off = false;
    on = true;

    forceableSwitch.update();

    forceableSwitch.toggleForcing(true);

    forceableSwitch.force(false);
    forceableSwitch.force(true);

    forceableSwitch.toggleForcing(false);

    off = false;
    on = true;

    forceableSwitch.update();

    assertTrue(isOn);
  }
}
