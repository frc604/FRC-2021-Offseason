// package robotnik.prefabs.motion;

// import com._604robotics.robotnik.prefabs.motion.AccelMotionState;
// import com._604robotics.robotnik.prefabs.motion.MotionConstraints;
// import com._604robotics.robotnik.prefabs.motion.MotionState;
// import com._604robotics.robotnik.prefabs.motion.SCurveMotionProfile;
// import java.awt.*;
// import java.util.ArrayList;
// import org.junit.jupiter.api.BeforeEach;
// import org.junit.jupiter.api.Test;
// import org.knowm.xchart.*;
// import org.knowm.xchart.style.Styler;

// public class SCurveMotionProfileTest {
//   private SCurveMotionProfile profile;

//   private MotionConstraints constraints;

//   @BeforeEach
//   void setUp() {
//     this.constraints = new MotionConstraints(30.0, 50.0, 29);
//     var start = System.nanoTime();
//     profile =
//         new SCurveMotionProfile(new MotionState(10.0, 5.0), new MotionState(20, 1.0), constraints);
//     System.out.println(System.nanoTime() - start);
//   }

//   @Test
//   void graph() {
//     ArrayList<Double> xData = new ArrayList<Double>();
//     ArrayList<Double> yData = new ArrayList<Double>();
//     ArrayList<Double> yPrimeData = new ArrayList<Double>();
//     ArrayList<Double> yDoublePrimeData = new ArrayList<Double>();

//     ArrayList<Long> times = new ArrayList<Long>();

//     var startLONG = System.nanoTime();
//     for (double i = 0; i <= profile.getTotalTime(); i = i + 0.0005) {
//       xData.add(i);
//       var start = System.nanoTime();
//       AccelMotionState state = profile.sample(i);
//       times.add(System.nanoTime() - start);

//       yData.add(state.velocity);
//       yPrimeData.add(state.position);
//       yDoublePrimeData.add(state.acceleration);
//     }
//     System.out.println(System.nanoTime() - startLONG);

//     long sum = 0;
//     for (Long d : times) sum += d;

//     System.out.println("AVERAGE" + sum / times.size());

//     XYChart chart =
//         new XYChartBuilder()
//             .width(800)
//             .height(600)
//             .title(getClass().getSimpleName())
//             .xAxisTitle("X")
//             .yAxisTitle("Y")
//             .build();

//     // Customize Chart
//     chart.getStyler().setLegendPosition(Styler.LegendPosition.InsideNE);
//     chart.getStyler().setAxisTitlesVisible(false);
//     chart.getStyler().setDefaultSeriesRenderStyle(XYSeries.XYSeriesRenderStyle.Area);

//     // Customize Chart
//     chart.getStyler().setPlotBackgroundColor(new Color(40, 42, 54));
//     chart.getStyler().setPlotGridLinesColor(new Color(68, 71, 90));
//     chart.getStyler().setChartBackgroundColor(new Color(46, 49, 62));
//     chart.getStyler().setPlotBorderColor(new Color(68, 71, 90));
//     chart.getStyler().setLegendBackgroundColor(new Color(68, 71, 90));
//     chart.getStyler().setAxisTickLabelsColor(new Color(248, 248, 242));
//     chart.getStyler().setAxisTickMarksColor(new Color(68, 71, 90));
//     chart.getStyler().setChartFontColor(new Color(248, 248, 242));
//     chart.getStyler().setChartTitleBoxVisible(false);
//     chart.getStyler().setPlotGridLinesVisible(true);

//     chart.getStyler().setLegendVisible(false);

//     // Series
//     var series1 = chart.addSeries("velocity", xData, yData);
//     series1.setXYSeriesRenderStyle(XYSeries.XYSeriesRenderStyle.Area);
//     series1.setLineColor(new Color(139, 233, 253));
//     series1.setMarkerColor(new Color(139, 233, 253));
//     series1.setFillColor(new Color(139, 233, 253, 100));

//     var series2 = chart.addSeries("position", xData, yPrimeData);
//     series2.setXYSeriesRenderStyle(XYSeries.XYSeriesRenderStyle.Line);
//     series2.setLineColor(new Color(255, 184, 108));
//     series2.setMarkerColor(new Color(255, 184, 108));

//     // var series3 = chart.addSeries("acceleration", xData, yDoublePrimeData);
//     // series3.setXYSeriesRenderStyle(XYSeries.XYSeriesRenderStyle.Line);
//     // series3.setLineColor(new Color(80, 250, 123));
//     // series3.setMarkerColor(new Color(80, 250, 123));

//     // new SwingWrapper(chart).displayChart();

//     // try {
//     //   Thread.sleep(2000000);
//     // } catch (InterruptedException e) {
//     //   e.printStackTrace();
//     // }
//   }
// }
