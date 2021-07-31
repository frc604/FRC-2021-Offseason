package robotnik.prefabs.vision;

import com._604robotics.robotnik.prefabs.vision.VisionCamera;
import java.util.ArrayList;
import java.util.List;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

public class TestCamera extends VisionCamera {
  private List<Target> targets = new ArrayList<>();

  public TestCamera(String name, Vector3D pose, double tilt) {
    super(name, pose, tilt);
  }

  public void addTarget(Target target) {
    targets.add(target);
  }

  @Override
  public PipelineVisionPacket getLatestMeasurement() {
    return new PipelineVisionPacket(!targets.isEmpty(), targets.get(0), targets, 0.0);
  }
}
