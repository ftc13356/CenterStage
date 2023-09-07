package org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController;

import static org.apache.commons.math3.util.FastMath.abs;
import static org.apache.commons.math3.util.FastMath.signum;

import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;

public class ControlPoints {
    ArrayList<ControlPoint> points;
    int controlPointRes = 100;
    double totalTime = 0;

    public ControlPoints(QuinticHermiteSpline p_spline, double p_v0, double p_v1) {
        points.add(new ControlPoint(p_spline.valsAt(0), p_v0, 0));
        points.get(0).setPrevK(points.get(0).getK());
        for (int i = 1; i < controlPointRes - 1; i++) {
            points.add(points.get(points.size() - 1).forwardTraverse(p_spline.valsAt((double) i / controlPointRes)));
            points.get(points.size() - 1).setPrevK(points.get(points.size() - 2).getK());
            points.get(points.size() - 2).setNextK(points.get(points.size() - 1).getK());
        }
        points.add(new ControlPoint(p_spline.valsAt(1), p_v1, 0));
        points.get(points.size() - 1).setPrevK(points.get(points.size() - 2).getK());
        points.get(points.size() - 2).setNextK(points.get(points.size() - 1).getK());
        points.get(points.size() - 1).setNextK(points.get(points.size() - 1).getK());
        double lastSignum = signum(points.get(points.size() - 1).getTime());
        for (int i = 1; i < controlPointRes - 1; i++) {
            int ind = controlPointRes - i;
            points.set(ind, points.get(ind + 1).backwardTraverse(points.get(ind)));
            double signum = signum(points.get(ind).getTime());
            if (signum != lastSignum && totalTime == 0) {
                totalTime = abs(points.get(ind).getTime()) + abs(points.get(ind).getOtherTime());
            }
            lastSignum = signum;
        }
    }

    /**
     * @param p_time
     * @return targetPose, targetVel, targetAccel, targetHead(vel, accel)
     */
    public Vector2d[] getTargets(double p_time) {
        double[] ind = bin(points, p_time);
        ControlPoint p1 = points.get((int) ind[0]), p2 = points.get((int) ind[0] + 1);
        Vector2d targetPose = p1.getPoseVec().times(ind[1]).plus(p2.getPoseVec().times(ind[2]));
        Vector2d targetVel = p1.getVeloVec().times(ind[1]).plus(p2.getVeloVec().times(ind[2]));
        Vector2d targetAccel = p1.getAccelVec().times(ind[1]).plus(p2.getAccelVec().times(ind[2]));
        Vector2d targetHead = p1.getHeadVec().times(ind[1]).plus(p2.getHeadVec().times(ind[2]));
        return new Vector2d[]{targetPose, targetVel, targetAccel, targetHead};
    }

    /**
     * binary search for first index of the range [a,a+1] that contains target time as well as the weights of how close it is to each border from 0-1
     *
     * @param p_arr array to search through
     * @param p_t   time to search for
     * @return array of {a, weight1, weight2}
     */
    public double[] bin(ArrayList<ControlPoint> p_arr, double p_t) {
        int upBound = p_arr.size() - 1, downBound = 0;
        while (upBound > downBound + 1) {
            int m = (upBound + downBound) / 2;
            if (p_arr.get(m).getTime() > p_t) {
                upBound = m;
            } else if (p_arr.get(m).getTime() < p_t) {
                downBound = m;
            } else {
                downBound = m;
                break;
            }
        }
        double downDist = p_t - p_arr.get(downBound).getTime();
        double upDist = p_arr.get(upBound).getTime() - p_t;
        double[] weight = {downDist / (downDist + upDist), upDist / (downDist + upDist)};
        return new double[]{downBound, weight[0], weight[1]};
    }

}
