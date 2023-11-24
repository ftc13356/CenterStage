package org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController;

import static org.apache.commons.math3.util.FastMath.pow;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.dashboard;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import static java.lang.Double.NaN;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;

public class QuinticHermiteSpline {
  private ArrayList<Vector2d> coeffs;
  private Pose2d startPose, endPose;

  public void initQuinticHermiteSpline(
      Pose2d p_startPos,
      Vector2d p_startVel,
      Vector2d p_startAccel,
      Pose2d p_endPos,
      Vector2d p_endVel,
      Vector2d p_endAccel) {
    startPose = p_startPos;
    endPose = p_endPos;
    coeffs = new ArrayList<>();
    coeffs.add(p_startPos.vec());
    coeffs.add(p_startVel);
    coeffs.add(p_startAccel.times(0.5));
    coeffs.add(
        p_startPos
            .vec()
            .times(-10)
            .plus(p_startVel.times(-6))
            .plus(p_startAccel.times(-1.5))
            .plus(p_endAccel.times(0.5))
            .plus(p_endVel.times(-4))
            .plus(p_endPos.vec().times(10)));
    coeffs.add(
        p_startPos
            .vec()
            .times(15)
            .plus(p_startVel.times(8))
            .plus(p_startAccel.times(1.5))
            .plus(p_endAccel.times(-1))
            .plus(p_endVel.times(7))
            .plus(p_endPos.vec().times(-15)));
    coeffs.add(
        p_startPos
            .vec()
            .times(-6)
            .plus(p_startVel.times(-3))
            .plus(p_startAccel.times(-0.5))
            .plus(p_endAccel.times(0.5))
            .plus(p_endVel.times(-3))
            .plus(p_endPos.vec().times(6)));
    for (int i = 0; i < coeffs.size(); i++) {
      packet.put("coeffs" + i, coeffs.get(i));
    }
  }

  public QuinticHermiteSpline(Pose2d[] p_coeffs) {
    initQuinticHermiteSpline(
        p_coeffs[0],
        p_coeffs[1].vec(),
        p_coeffs[2].vec(),
        p_coeffs[3],
        p_coeffs[4].vec(),
        p_coeffs[5].vec());
  }

  public QuinticHermiteSpline(
      Pose2d p_startPos,
      Vector2d p_startVel,
      Vector2d p_startAccel,
      Pose2d p_endPos,
      Vector2d p_endVel,
      Vector2d p_endAccel) {
    initQuinticHermiteSpline(p_startPos, p_startVel, p_startAccel, p_endPos, p_endVel, p_endAccel);
  }

  /**
   * Calculates the important values on a spline at this point in time
   *
   * @param p_t what time parameter it is
   * @return x, dx, ddx
   */
  public Pose2d[] valsAt(double p_t) {
    Vector2d[] vals = {new Vector2d(0, 0), new Vector2d(0, 0), new Vector2d(0, 0)};
    for (int i = 0; i < coeffs.size(); i++) {
      vals[0] = vals[0].plus(coeffs.get(i).times(pow(p_t, i)));
    }
    for (int i = 1; i < coeffs.size(); i++) {
      vals[1] = vals[1].plus(coeffs.get(i).times(pow(p_t, i - 1)).times(i));
    }
    for (int i = 2; i < coeffs.size(); i++) {
      vals[2] = vals[2].plus(coeffs.get(i).times(pow(p_t, i - 2)).times(i * (i - 1)));
    }
    packet.put("vals", vals[0] + ", T = " + p_t);
    //    dashboard.sendTelemetryPacket(packet);
    Pose2d[] vals2 = {new Pose2d(), new Pose2d(), new Pose2d()};
    vals2[0] =
        new Pose2d(
            vals[0], vals[1].angle() + (startPose.getHeading() - endPose.getHeading()) * (1 - p_t));
    double k =
        (vals[1].getX() * vals[2].getY() - vals[1].getY() * vals[2].getX())
            / pow(vals[1].getX() * vals[1].getX() + vals[1].getY() * vals[1].getY(), 1.5);
    if (Double.isNaN(k)) {
      k = 0;
    }
    vals2[1] = new Pose2d(vals[1], k - (startPose.getHeading() - endPose.getHeading()));
    return vals2;
  }

  @SuppressLint("NewApi")
  public Object[] binRecursion() {
    ArrayList<Double> times = new ArrayList<>();
    times.add((double) 0);
    times.add((double) 0.5);
    times.add((double) 1);
    times = binRecurse(times, 1);
    times = binRecurse(times, 2);
    return times.toArray();
  }

  public ArrayList<Double> binRecurse(ArrayList<Double> p_times, int p_index) {
    if (p_index != 0) {
      Pose2d p1 = valsAt(p_times.get(p_index - 1))[0];
      Pose2d p2 = valsAt(p_times.get(p_index))[0];
      if (p2.vec().distTo(p1.vec())
              + (p_times.get(p_index) - p_times.get(p_index - 1))
                  * (startPose.getHeading() - endPose.getHeading())
                  * 6
          < 1) {
        p_times.add(p_index - 1, (p_times.get(p_index) + p_times.get(p_index - 1)) * 0.5);
        binRecurse(p_times, p_index);
      }
      return p_times;
    }
    return p_times;
  }
}
