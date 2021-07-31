// package com._604robotics.quixsam;

// import java.util.HashMap;

// import com._604robotics.quixsam.mathematics.DoubleInterpolatableTreeMap;
// import com._604robotics.quixsam.mathematics.Interpolatable;
// import com._604robotics.quixsam.odometry.DiffDriveOdometryMeasurement;
// import com._604robotics.quixsam.odometry.VisionMeasurement;
// import com._604robotics.robotnik.prefabs.vision.VisionCamera;

// import edu.wpi.first.wpilibj.RobotController;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.geometry.Pose2d;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
// import edu.wpi.first.wpilibj.kinematics.MecanumDriveOdometry;
// import edu.wpi.first.wpiutil.math.Pair;

// public class GTSAMLocalizer {
//     private int currentID = 0;
//     private HashMap<Integer, Double> idMap;

//     private HashMap<Double, DiffDriveOdometryMeasurement> odometryMap;
//     private DoubleInterpolatableTreeMap<Pose2d> poseMap = new DoubleInterpolatableTreeMap<>();

//     private DifferentialDriveOdometry rawOdometry;
//     private DifferentialDriveOdometry playbackOdometry;

//     public GTSAMLocalizer(Pose2d poseMeters, Rotation2d gyroAngle) {
//         rawOdometry = new DifferentialDriveOdometry(gyroAngle, poseMeters);
//         playbackOdometry = new DifferentialDriveOdometry(gyroAngle, poseMeters);
//     }

//     public void update(DiffDriveOdometryMeasurement odometry, VisionCamera.PipelineVisionPacket
// vision) {
//         currentID += 1;
//         double currentTime = Timer.getFPGATimestamp();

//         odometry.addTo(rawOdometry);
//         odometry.addTo(playbackOdometry);
//         odometryMap.put(currentTime, odometry);

//         poseMap.set(currentTime, Interpolatable.interPose2d(rawOdometry.getPoseMeters()));

//         double visionTime = currentTime - (vision.getLatency() / 1000);
//         Pose2d interpolatedPose = poseMap.get(visionTime);

//         idMap.put(currentID, visionTime);

//         // Send [currentID, interpolatedPose + bearingElevationVision] to GTSAM
//         currentID += 1;
//         idMap.put(currentID, currentTime);
//         // Send [currentID, rawOdometry.pose()] to GTSAM
//     }

//     public void update(DiffDriveOdometryMeasurement odometry) {
//         currentID += 1;
//         double currentTime = Timer.getFPGATimestamp();

//         odometry.addTo(rawOdometry);
//         odometry.addTo(playbackOdometry);
//         odometryMap.put(currentTime, odometry);

//         poseMap.set(currentTime, Interpolatable.interPose2d(rawOdometry.getPoseMeters()));

//         idMap.put(currentID, currentTime);
//         // Send [currentID, rawOdometry.pose()] to GTSAM
//     }

//     public void computeEstimate(newGTSAMEstimate) {
//         int estimateID = newGTSAMEstimate.getID()
//         estimateTime = idMap.getValue(estimateID);

//         playbackOdometry.reset(newGTSAMEstimate.pose);

//         Double lastKey = odometryMap.ceilingKey(estimateTime); //least key >= time

//         while (lastKey != null) {
//             playbackOdometry.update(odometryMap.get(lastKey));
//             odometryMap.remove(lastKey);
//             lastKey = odometryMap.ceilingKey(lastKey);
//         }
//     }

//     public Pose2d getPose() {
//         return playbackOdometry.getPoseMeters();
//     }
// }
