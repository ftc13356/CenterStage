package org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController;

import static org.apache.commons.math3.util.FastMath.pow;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.ArrayList;

public class CubicHermiteSpline {
  ArrayList<Pose2d> coeffs;

  public CubicHermiteSpline(
      Pose2d currentPose, Pose2d currentVelo, Pose2d endPose, Pose2d endVelo) {
    coeffs.add(currentPose);
    coeffs.add(currentVelo);
    coeffs.add(
        currentPose
            .times(-3)
            .plus(currentVelo.times(-1))
            .plus(endPose.times(3))
            .plus(endVelo.times(-1)));
    coeffs.add(currentPose.times(2).plus(currentVelo).plus(endVelo).plus(endPose.times(-2)));
  }

  public Pose2d initRatio() {
    Pose2d vals = new Pose2d(0, 0, 0);
    for (int i = 2; i < coeffs.size(); i++) {
      vals = vals.plus(coeffs.get(i).times(pow(0, i - 2)).times(i * (i - 1)));
    }
    return vals;
  }
}
