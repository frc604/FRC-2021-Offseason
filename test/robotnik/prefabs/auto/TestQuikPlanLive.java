// package robotnik.prefabs.auto;

// import com._604robotics.robotnik.prefabs.auto.QuikPlanLive;
// import com._604robotics.robotnik.prefabs.auto.QuikPlanLive.TrajectoryState;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj.geometry.Pose2d;
// import edu.wpi.first.wpilibj.geometry.Rotation2d;
// import edu.wpi.first.wpilibj.geometry.Translation2d;
// import edu.wpi.first.wpilibj.util.Units;
// import java.awt.BasicStroke;
// import java.awt.Color;
// import java.awt.Graphics2D;
// import java.util.ArrayList;
// import java.util.Arrays;
// import java.util.List;
// import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.api.Test;
// import org.knowm.xchart.SwingWrapper;
// import org.knowm.xchart.XYChart;
// import org.knowm.xchart.XYChartBuilder;
// import org.knowm.xchart.XYSeries.XYSeriesRenderStyle;
// import org.knowm.xchart.style.markers.Marker;
// import org.knowm.xchart.style.markers.SeriesMarkers;

// public class TestQuikPlanLive {
//   private QuikPlanLive live;

//   @BeforeEach
//   void setUp() {
//     NetworkTableInstance.getDefault().startClient("localhost");
//     live = new QuikPlanLive();
//   }

//   @Test
//   void testLoad() throws InterruptedException {
//     Thread.sleep(5000);
//     List<Translation2d> targets =
//         Arrays.asList(
//             new Translation2d(Units.inchesToMeters(30), Units.inchesToMeters(30)),
//             new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(60)),
//             new Translation2d(Units.inchesToMeters(120), Units.inchesToMeters(120)));

//     live.publishObstacles(targets, Units.inchesToMeters(7.0 / 2.0));
//     while (!live.isTrajectoryValid()) {
//       System.out.println("Waiting...");
//       Thread.sleep(1000);
//     }
//     live.loadTrajectory();

//     ArrayList<Pose2d> robotPoses = new ArrayList<Pose2d>();
//     ArrayList<Translation2d> targetTranslations = new ArrayList<Translation2d>();

//     targetTranslations.addAll(targets);

//     for (double i = 0; i <= live.getTotalTime(); i += 0.02) {
//       List<Double> state = live.getState(i);
//       robotPoses.add(
//           new Pose2d(
//               state.get(TrajectoryState.x.index),
//               state.get(TrajectoryState.y.index),
//               new Rotation2d(state.get(TrajectoryState.Theta.index))));
//     }

//     XYChart chart =
//         new XYChartBuilder()
//             .width(800)
//             .height(600)
//             .title(getClass().getSimpleName())
//             .xAxisTitle("X")
//             .yAxisTitle("Y")
//             .build();

//     chart.getStyler().setDefaultSeriesRenderStyle(XYSeriesRenderStyle.Scatter);
//     chart.getStyler().setLegendVisible(false);
//     chart.getStyler().setMarkerSize(20);

//     for (Pose2d pose : robotPoses) {
//       chart
//           .addSeries(
//               String.valueOf(pose.hashCode()),
//               new double[] {pose.getX()},
//               new double[] {pose.getY()})
//           .setMarker(new PoseMarker(pose.getRotation(), Color.RED, Color.BLUE));
//     }

//     for (Translation2d translation : targetTranslations) {
//       chart
//           .addSeries(
//               translation.toString(),
//               new double[] {translation.getX()},
//               new double[] {translation.getY()})
//           .setMarker(SeriesMarkers.TRIANGLE_UP);
//     }

//     new SwingWrapper(chart).displayChart();

//     try {
//       Thread.sleep(2000000);
//     } catch (InterruptedException e) {
//       e.printStackTrace();
//     }
//   }

//   public class PoseMarker extends Marker {
//     private Rotation2d rotation;
//     private Color xColor;
//     private Color yColor;

//     public PoseMarker(Rotation2d rotation, Color xColor, Color yColor) {
//       this.rotation = rotation;
//       this.xColor = xColor;
//       this.yColor = yColor;
//     }

//     @Override
//     public void paint(Graphics2D g, double xOffset, double yOffset, int markerSize) {
//       g.setStroke(new BasicStroke(5, BasicStroke.CAP_ROUND, BasicStroke.CAP_BUTT));
//       // Note y-direction for markers is inverted.
//       g.setColor(yColor);
//       g.drawLine(
//           (int) xOffset,
//           (int) yOffset,
//           (int) (xOffset + markerSize * rotation.getSin()),
//           (int) (yOffset + markerSize * rotation.getCos()));
//       g.setColor(xColor);
//       g.drawLine(
//           (int) xOffset,
//           (int) yOffset,
//           (int) (xOffset + markerSize * rotation.getCos()),
//           (int) (yOffset - markerSize * rotation.getSin()));
//     }
//   }
// }
