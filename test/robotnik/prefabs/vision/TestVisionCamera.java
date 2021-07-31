package robotnik.prefabs.vision;

import com._604robotics.quixsam.mathematics.Interpolatable;
import com._604robotics.robot2020.constants.Calibration;
import com._604robotics.robotnik.prefabs.vision.VisionCamera.GamePiece;
import com._604robotics.robotnik.prefabs.vision.VisionCamera.Target;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpiutil.math.Pair;
import java.util.List;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

public class TestVisionCamera {
  private TestCamera camera;

  @BeforeEach
  void setUp() {
    camera = new TestCamera("camera", new Vector3D(-0.10813321, 0.01653152, 1.03617424), -20);
  }

  @Test
  void testSearch() {
    camera.addTarget(new Target(0, -45, 0, 0));
    camera.pull();
    System.out.println(camera.searchGamePiece(new Pose2d(), GamePiece.THREEPOINTFIVE));
  }

  @Test
  void testGetObs() {
    Calibration.Auto.XDISTACEBALLCAM.set(
        new Pair<Double, Interpolatable<Double>>(9.543333, Interpolatable.interDouble(6.096)),
        new Pair<Double, Interpolatable<Double>>(9.036667, Interpolatable.interDouble(5.7912)),
        new Pair<Double, Interpolatable<Double>>(8.54, Interpolatable.interDouble(5.4864)),
        new Pair<Double, Interpolatable<Double>>(7.96, Interpolatable.interDouble(5.1816)),
        new Pair<Double, Interpolatable<Double>>(7.27, Interpolatable.interDouble(4.8768)),
        new Pair<Double, Interpolatable<Double>>(5.75, Interpolatable.interDouble(4.572)),
        new Pair<Double, Interpolatable<Double>>(5.1, Interpolatable.interDouble(4.2672)),
        new Pair<Double, Interpolatable<Double>>(4.06667, Interpolatable.interDouble(3.9624)),
        new Pair<Double, Interpolatable<Double>>(3.05667, Interpolatable.interDouble(3.6576)),
        new Pair<Double, Interpolatable<Double>>(1.88667, Interpolatable.interDouble(3.3528)),
        new Pair<Double, Interpolatable<Double>>(0.51, Interpolatable.interDouble(3.048)),
        new Pair<Double, Interpolatable<Double>>(-1.68, Interpolatable.interDouble(2.7432)),
        new Pair<Double, Interpolatable<Double>>(-3.7733, Interpolatable.interDouble(2.4384)),
        new Pair<Double, Interpolatable<Double>>(-6.4367, Interpolatable.interDouble(2.1336)),
        new Pair<Double, Interpolatable<Double>>(-10.877, Interpolatable.interDouble(1.8288)),
        new Pair<Double, Interpolatable<Double>>(-15.513, Interpolatable.interDouble(1.524)));

    camera.addTarget(new Target(0, -45, 0, 0));
    // camera.addTarget(new Target(9.18, 6.45, 0, 0));
    // camera.addTarget(new Target(-3.33, 7.52, 0, 0));
    camera.pull();
    List<Translation2d> pieces =
        camera.searchGamePiece(
            new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180)), GamePiece.THREEPOINTFIVE);
    String output = "";
    for (Translation2d piece : pieces) {
      output = output + "[" + piece.getX() + ", " + piece.getY() + "]";
    }
    System.out.println(output);
  }
}
