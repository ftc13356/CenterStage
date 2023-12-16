package org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController;

import static org.apache.commons.math3.util.FastMath.abs;
import static org.apache.commons.math3.util.FastMath.pow;
import static org.apache.commons.math3.util.FastMath.signum;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import android.annotation.SuppressLint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;

public class CubicHermiteSpline {
  ArrayList<Pose2d> coeffs;
  ArrayList<ControlPoint> points;
  ArrayList<Double> times;

  Pose2d endVelo;
  double totalTime = 0;

  public CubicHermiteSpline(
      Pose2d currentPose, Pose2d currentVelo, Pose2d endPose, Pose2d endVelo) {
    coeffs = new ArrayList<>();
    points = new ArrayList<>();
    coeffs.add(currentPose);
    coeffs.add(currentVelo);
    coeffs.add(
        currentPose
            .times(-3)
            .plus(currentVelo.times(-1))
            .plus(endPose.times(3))
            .plus(endVelo.times(-1)));
    coeffs.add(currentPose.times(2).plus(currentVelo).plus(endVelo).plus(endPose.times(-2)));
    this.endVelo = endVelo;
    times = new ArrayList<>();
  }
  public Pose2d[] valsAt(double p_t) {
    Pose2d[] vals = {new Pose2d(0, 0), new Pose2d(0, 0), new Pose2d(0, 0)};
    for (int i = 0; i < coeffs.size(); i++) {
      vals[0] = vals[0].plus(coeffs.get(i).times(pow(p_t, i)));
    }
    for (int i = 1; i < coeffs.size(); i++) {
      vals[1] = vals[1].plus(coeffs.get(i).times(pow(p_t, i - 1)).times(i));
    }
    for (int i = 2; i < coeffs.size(); i++) {
      vals[2] = vals[2].plus(coeffs.get(i).times(pow(p_t, i - 2)).times(i * (i - 1)));
    }
    double k =
            (vals[1].getX() * vals[2].getY() - vals[1].getY() * vals[2].getX())
                    / pow(vals[1].getX() * vals[1].getX() + vals[1].getY() * vals[1].getY(), 1.5);
    if (Double.isNaN(k)) {
      k = 0;
    }
    vals[1] = new Pose2d(vals[1].vec(), k);
    packet.put("vals", vals[0] + ", T = " + p_t);
    return vals;
  }
  @SuppressLint("NewApi")
  public Object[] binRecursion() {
    times.add((double) 0);
    times.add((double) 0.5);
    times.add((double) 1);
    binRecurse( 1);
    binRecurse( times.size()-1);
    return times.toArray();
  }

  public void binRecurse(int p_index) {
    int backIndex = times.size()-p_index;
    if (p_index != 0) {
      Pose2d p1 = valsAt(times.get(p_index - 1))[0];
      Pose2d p2 = valsAt(times.get(p_index))[0];
      while (p2.vec().distTo(p1.vec())> 1) {
        times.add(p_index, (times.get(p_index) + times.get(p_index - 1)) * 0.5);
        binRecurse(p_index);
        p_index = times.size() - backIndex;
        p1 = valsAt(times.get(p_index - 1))[0];
        p2 = valsAt(times.get(p_index))[0];
      }
    }
  }
  public ControlPoint initRatio() {
    Pose2d vals = new Pose2d(0, 0, 0);
    Pose2d p_v0 = currentVelocity;
    Pose2d p_v1 = endVelo;
    points.add(new ControlPoint(valsAt(0), currentVelocity.vec().norm(), 0, currentVelocity.getHeading()));
    points = new ArrayList<>();
    points.add(new ControlPoint(valsAt(0), p_v0.vec().norm(), 0, p_v0.getHeading()));
    points.get(0).setPrevK(points.get(0).getK());
    Object[] times = binRecursion();
    for (int i = 1; i < times.length-1; i++) {
      points.add(points.get(points.size() - 1).forwardTraverse(valsAt((double)times[i])));
      points.get(points.size() - 1).setPrevK(points.get(points.size() - 2).getK());
      points.get(points.size() - 2).setNextK(points.get(points.size() - 1).getK());
    }
    points.add(new ControlPoint(valsAt(1), p_v1.vec().norm(), 0, p_v1.getHeading()));
    points.get(points.size() - 1).setPrevK(points.get(points.size() - 2).getK());
    points.get(points.size() - 2).setNextK(points.get(points.size() - 1).getK());
    points.get(points.size() - 1).setNextK(points.get(points.size() - 1).getK());
    double lastSignum = signum(points.get(points.size() - 1).getTime());
    for (int i = 2; i < times.length; i++) {
      int ind = times.length - i;
      points.set(ind, points.get(ind + 1).backwardTraverse(points.get(ind)));
      double signum = signum(points.get(ind).getTime());
      if (signum != lastSignum && totalTime == 0) {
        totalTime = abs(points.get(ind).getTime()) + abs(points.get(ind).getOtherTime());
      }
      lastSignum = signum;
    }
    return points.get(1);
  }
}
