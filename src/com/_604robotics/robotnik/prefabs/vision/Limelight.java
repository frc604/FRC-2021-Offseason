package com._604robotics.robotnik.prefabs.vision;

import java.util.ArrayList;
import java.util.List;
import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpiutil.math.Pair;

public class Limelight extends VisionCamera {
  private final NetworkTable limelightTable;

  public Limelight(String name, Vector3D pose, double tilt) {
    super(name, pose, tilt);
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
  }

  @Override
  public PipelineVisionPacket getLatestMeasurement() {
    boolean hasTargets = (int) limelightTable.getEntry("tv").getNumber(0) == 1;

    if (hasTargets) {
      double yaw = limelightTable.getEntry("tx").getDouble(0);
      double pitch = limelightTable.getEntry("ty").getDouble(0);
      double area = limelightTable.getEntry("ta").getDouble(0);
      double skew = limelightTable.getEntry("ts").getDouble(0);
      double latency = limelightTable.getEntry("tl").getDouble(0) + 11; // Magic number from LL website.

      double[] cornersX = limelightTable.getEntry("tcornx").getDoubleArray(new double[0]);
      double[] cornersY = limelightTable.getEntry("tcorny").getDoubleArray(new double[0]);

      ArrayList<Pair<Double, Double>> corners = new ArrayList<>();
      for (int i = 0; i < cornersX.length; i++) {
        corners.add(new Pair<Double,Double>(cornersX[i], cornersY[i]));
      }

      Target target = new Target(yaw, pitch, area, skew, corners);
      List<Target> targets = new ArrayList<>();
      targets.add(target);

      return new PipelineVisionPacket(
          hasTargets,
          target,
          targets,
          latency);
    } else {
      return new PipelineVisionPacket(false, new Target(), new ArrayList<Target>(), 0.0);
    }
  }
}
