package org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController;

import static org.apache.commons.math3.util.FastMath.abs;
import static org.apache.commons.math3.util.FastMath.signum;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentVelocity;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;

public class ControlPoints {
    ArrayList<ControlPoint> points;
    int controlPointRes = 100;
    double totalTime = 0;
    double startTime =0;

    public ControlPoints(QuinticHermiteSpline p_spline, double p_v0, double p_v1) {
        startTime = time;
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
     * targetPose, targetVel, targetAccel, targetHead(vel, accel)
     * @param p_time time
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

    public Pose2d getTargetPoint(){
        return points.get(points.size()-1).getPose();
    }
    public Vector2d getTargetVel(){
        return points.get(points.size()-1).getVeloVec();
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
            double t = p_arr.get(m).getTime();
            if(t<0){
                t = startTime+totalTime-t;
            }else{
                t = startTime+t;
            }
            if (t> p_t) {
                upBound = m;
            } else if (t< p_t) {
                downBound = m;
            } else {
                downBound = m;
                break;
            }
        }
        double tDown = p_arr.get(downBound).getTime();
        double tUp = p_arr.get(upBound).getTime();


        double downDist = p_t - tDown;
        double upDist = tUp - p_t;
        double[] weight = {downDist / (downDist + upDist), upDist / (downDist + upDist)};
        return new double[]{downBound, weight[0], weight[1]};
    }

    public double[] binPose(ArrayList<ControlPoint> p_arr, Vector2d p_pose) {
        int upBound = p_arr.size() - 1, downBound = 0;
        while (upBound > downBound + 1) {
            int m = (upBound + downBound) / 2;
            if (p_arr.get(m).getPoseVec().distTo(p_pose) > p_arr.get(m+1).getPoseVec().distTo(p_pose)) {
                downBound = m;
            } else if (p_arr.get(m).getPoseVec().distTo(p_pose) < p_arr.get(m+1).getPoseVec().distTo(p_pose)) {
                upBound = m;
            } else {
                downBound = m;
                break;
            }
        }
        double downDist = p_pose.distTo(p_arr.get(downBound).getPoseVec());
        double upDist = p_pose.distTo(p_arr.get(upBound).getPoseVec());
        double[] weight = {downDist / (downDist + upDist), upDist / (downDist + upDist)};
        return new double[]{downBound, weight[0], weight[1]};
    }

    public Pose2d[] getInstantaneousTarget(){
        double[] ind = binPose(points, currentPose.vec());
        ArrayList<ControlPoint> points2 = new ArrayList<>();
        QuinticHermiteSpline spline = new QuinticHermiteSpline(currentPose.vec(),currentVelocity.vec(),points.get((int)ind[0]).getAccelVec(),
                getTargetPoint().vec(), getTargetVel(), points.get(points.size()-1).getAccelVec());
        points2.add(new ControlPoint(spline.valsAt(0), currentVelocity.vec().norm(), 0));
        points2.get(0).setPrevK(points.get(0).getK());
        for (int i = 1; i < 3; i++) {
            points2.add(points2.get(points2.size() - 1).forwardTraverse(spline.valsAt((double) i / controlPointRes)));
            points2.get(points2.size() - 1).setPrevK(points2.get(points2.size() - 2).getK());
            points2.get(points2.size() - 2).setNextK(points2.get(points2.size() - 1).getK());
        }
        points2.set(1, points.get((int)(ind[0]+2)).backwardTraverse(points2.get(1)));
        Pose2d targetPose = points2.get(1).getPose();
        Pose2d targetVel = new Pose2d(points2.get(1).getVeloVec(),points2.get(1).getK());
        Pose2d targetAccel = new Pose2d(points2.get(1).getAccelVec(), points2.get(1).getKAccel());
        return new Pose2d[]{targetPose,targetVel,targetAccel};
    }

}
