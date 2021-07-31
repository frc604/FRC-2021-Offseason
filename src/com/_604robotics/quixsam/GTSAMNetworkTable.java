// package com._604robotics.quixsam;

// import java.util.ArrayList;
// import java.util.Arrays;

// import com._604robotics.quixsam.odometry.DiffDriveOdometryMeasurement;

// import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.geometry.Pose2d;

// public class GTSAMNetworkTable {
//     private NetworkTable quixsamTable;
//     private NetworkTable visionTable;
//     private NetworkTable odometryTable;
//     private NetworkTable landmarksTable;
//     private NetworkTable prioriTable;
//     private NetworkTable estimatesTable;

//     private Pose2d latestEstimate = new Pose2d();

//     public GTSAMNetworkTable(Pose2d priori) {
//         quixsamTable = NetworkTableInstance.getDefault().getTable("quixsam");
//         visionTable = quixsamTable.getSubTable("vision");
//         odometryTable = quixsamTable.getSubTable("odometry");
//         landmarksTable = quixsamTable.getSubTable("landmarks");
//         prioriTable = quixsamTable.getSubTable("priori");
//         estimatesTable = quixsamTable.getSubTable("estimates");

//         Double[] emptyOdometry = new Double[7];
//         Arrays.fill(emptyOdometry, 0.0);
//         odometryTable.getEntry("0").setNumberArray(emptyOdometry);

//         Double[] emptyVision = new Double[6];
//         Arrays.fill(emptyVision, 0.0);
//         visionTable.getEntry("0").setNumberArray(emptyVision);
//     }

//     public void addLandmark(int id, Vector3D pose, Vector3D sigmas) {
//         Double[] landmarkData = new Double[]{(double) id, pose.getX(), pose.getY(), pose.getZ(),
// sigmas.getX(), sigmas.getY(), sigmas.getZ()};
//         landmarksTable.getEntry(String.valueOf(id)).setNumberArray(landmarkData);
//     }

//     public void publishOdometry(int id, Pose2d pose, Pose2d sigmas) {
//         Double[] odometryData = new Double[]{(double) id, pose.getX(), pose.getY(),
// pose.getRotation().getRadians(), sigmas.getX(), sigmas.getY(),
// sigmas.getRotation().getRadians()};
//         odometryTable.getEntry(String.valueOf(id)).setNumberArray(odometryData);
//     }

//     public void publishVision(int id, int landmarkID, double bearing, double elevation, double
// sigmaBearing, double sigmaElevation) {
//         Double[] visionData = new Double[]{(double) id, (double) landmarkID, bearing, elevation,
// sigmaBearing, sigmaElevation};
//         visionTable.getEntry(String.valueOf(id)).setNumberArray(visionData);
//     }

//     public void update() {

//     }

// }
