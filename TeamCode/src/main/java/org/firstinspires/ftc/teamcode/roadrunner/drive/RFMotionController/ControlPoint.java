package org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController;

import static org.apache.commons.math3.util.FastMath.abs;
import static org.apache.commons.math3.util.FastMath.asin;
import static org.apache.commons.math3.util.FastMath.atan2;
import static org.apache.commons.math3.util.FastMath.cos;
import static org.apache.commons.math3.util.FastMath.max;
import static org.apache.commons.math3.util.FastMath.pow;
import static org.apache.commons.math3.util.FastMath.sin;
import static org.apache.commons.math3.util.FastMath.sqrt;
import static org.apache.commons.math3.util.MathUtils.min;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.TRACK_WIDTH;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.apache.commons.math3.util.FastMath;

import java.util.ArrayList;

public class ControlPoint {
  private final double x, y, t, k, v, a, time, otherTime, h, w, ah;
  private double prevK = 0, nextK = 0, deltaT = 0;

  public ControlPoint(
      Pose2d[] p_vals, double p_v, double p_time, double p_w) {
    x = p_vals[0].getX();
    y = p_vals[0].getY();
    t = p_vals[1].vec().angle();
    h = p_vals[0].getHeading();
    k = p_vals[1].getHeading();
    time = p_time;
    v = p_v;
    a = 0;
    otherTime = 0;
    w = p_w;
    ah=0;
  }

  public ControlPoint(
      double p_x,
      double p_y,
      double p_t,
      double p_k,
      double p_v,
      double p_a,
      double p_time,
      double p_h,
      double p_w,
      double p_ah) {
    x = p_x;
    y = p_y;
    t = p_t;
    k = p_k;
    v = p_v;
    a = p_a;
    time = p_time;
    otherTime = 0;
    h = p_h;
    w = p_w;
    ah = p_ah;
  }

  private ControlPoint(
      double p_x,
      double p_y,
      double p_t,
      double p_k,
      double p_v,
      double p_a,
      double p_time,
      double p_otherTime,
      double p_filler,
      double p_h,
      double p_w,
      double p_ah) {
    x = p_x;
    y = p_y;
    t = p_t;
    k = p_k;
    v = p_v;
    a = p_a;
    time = p_time;
    otherTime = p_otherTime;
    h = p_h;
    w = p_w;
    ah = p_ah;
    deltaT = p_filler;
  }

  public ControlPoint forwardTraverse(Pose2d[] p_vals) {
    double n_t = atan2(p_vals[1].getY(), p_vals[1].getX());
    double n_h = p_vals[0].getHeading();
    double n_k = p_vals[1].getHeading();
    double dh = (n_h - h) * TRACK_WIDTH;
    double ab = w + v;

    double avgK = (n_k + k) * 0.5;

    // n_h = wt + 0.5 * ah * t^2, max_a = ah + sqrt(ax^2 + ay^2)
    // d = 0.5ax^2
    double dist =
        sqrt(
            (p_vals[0].getX() - x) * (p_vals[0].getX() - x)
                + (p_vals[0].getY() - y) * (p_vals[0].getY() - y));
    double radians = 2 * asin(dist * avgK * 0.5);
    double arcLength = radians / avgK;
    double deldaT =
        (ab + sqrt(ab * ab + 2 * (MAX_ACCEL - v * v * avgK) * (dh + arcLength)))
            / (MAX_ACCEL - v * v * avgK);
    double a_t = 2 * (arcLength - v * deldaT) / (deldaT * deldaT);
    double a_h = 2 * (dh - w * TRACK_WIDTH * deldaT) / (deldaT * deldaT);
    double a_c = v * v * n_k;
    double scaleDown = 1 / (a_t + a_h + a_c);
    a_t *= scaleDown;
    a_c *= scaleDown;
    a_h *= scaleDown;
    double n_time = time + deldaT;
    double n_v = FastMath.min(v + a_t * deldaT, MAX_VEL);
    double n_w = FastMath.min(w + a_h * deldaT, MAX_ANG_VEL);
    double n_a = a_t;
    return new ControlPoint(
        p_vals[0].getX(), p_vals[0].getY(), n_t, n_k, n_v, n_a, n_time, 0, deldaT, n_h, n_w, a_h);
  }

  public ControlPoint backwardTraverse(ControlPoint prev) {
    double dh = (prev.h - h) * TRACK_WIDTH;
    double ab = w + v;
    double avgK = (prev.k + k) * 0.5;

    // n_h = wt + 0.5 * ah * t^2, max_a = ah + sqrt(ax^2 + ay^2)
    // d = 0.5ax^2
    double dist = sqrt((prev.x - x) * (prev.x - x) + (prev.y - y) * (prev.y - y));
    double radians = 2 * asin(dist * avgK * 0.5);
    double arcLength = radians / avgK;
    double deldaT =
        (ab + sqrt(ab * ab + 2 * (MAX_ACCEL - v * v * avgK) * (dh + arcLength)))
            / (MAX_ACCEL - v * v * avgK);
    double a_t = 2 * (arcLength - v * deldaT) / (deldaT * deldaT);
    double a_h = 2 * (dh - w * TRACK_WIDTH * deldaT) / (deldaT * deldaT);
    double a_c = v * v * prev.k;
    double scaleDown = 1 / (a_t + a_h + a_c);
    a_t *= scaleDown;
    a_c *= scaleDown;
    a_h *= scaleDown;
    double n_time, o_time;
    double n_v = FastMath.min(v + a_t * deldaT, prev.v);
    double n_w = FastMath.min(w + a_h * deldaT, prev.w);
    double n_a = a_t;
    if (deldaT <= prev.deltaT) {
      n_v = prev.v;
      n_time = prev.time;
      o_time = time - deldaT;
    } else {
      n_time = time - deldaT;
      o_time = prev.time;
    }
    return new ControlPoint(
        prev.x, prev.y, prev.t, prev.k, n_v, n_a, n_time, o_time, deldaT, prev.h, n_w, a_h);
  }

  public double getCentripetalAccel() {
    return v * v * k;
  }

  public void setPrevK(double p_k) {
    prevK = p_k;
  }

  public void setNextK(double p_k) {
    nextK = p_k;
  }

  public Pose2d getPose() {
    return new Pose2d(x, y, t);
  }

  public Pose2d getVelo() {
    return new Pose2d(v * cos(t), v * sin(t), v * k);
  }

  public Pose2d getAccel() {
    double c = getCentripetalAccel();
    return new Pose2d(a * cos(t) - c * sin(t), a * sin(t) - c * cos(t), v * (nextK - k));
  }

  public Vector2d getPoseVec() {
    return new Vector2d(x, y);
  }

  public Vector2d getVeloVec() {
    return new Vector2d(v * cos(t), v * sin(t));
  }

  public Vector2d getAccelVec() {
    double c = getCentripetalAccel();
    return new Vector2d(a * cos(t) - c * sin(t), a * sin(t) - c * cos(t));
  }

  public Vector2d getHeadVec() {
    return new Vector2d(k, (nextK - prevK) * 0.5);
  }

  public double getX() {
    return x;
  }

  public double getTime() {
    return time;
  }

  public double getOtherTime() {
    return otherTime;
  }

  public double getY() {
    return y;
  }

  public double getT() {
    return t;
  }

  public double getK() {
    return k;
  }

  public double getKAccel() {
    return nextK - k;
  }

  public double getV() {
    return v;
  }

  public double getA() {
    return a;
  }
}
