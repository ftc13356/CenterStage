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
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MAX_VEL;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.apache.commons.math3.util.FastMath;

import java.util.ArrayList;

public class ControlPoint {
    private final double x, y, t, k, v, a, time, otherTime;
    private double prevK=0, nextK=0;


    public ControlPoint(Vector2d[] p_vals, double p_v, double p_time) {
        x = p_vals[0].getX();
        y = p_vals[0].getY();
        t = atan2(p_vals[1].getY(), p_vals[1].getX());
        k = (p_vals[1].getX() * p_vals[2].getY() - p_vals[1].getY() * p_vals[2].getX()) / pow(p_vals[1].getX() * p_vals[1].getX() + p_vals[1].getY() * p_vals[1].getY(), 1.5);
        time = p_time;
        v = p_v;
        a = 0;
        otherTime = 0;
    }

    public ControlPoint(double p_x, double p_y, double p_t, double p_k, double p_v, double p_a, double p_time) {
        x = p_x;
        y = p_y;
        t = p_t;
        k = p_k;
        v = p_v;
        a = p_a;
        time = p_time;
        otherTime = 0;
    }

    private ControlPoint(double p_x, double p_y, double p_t, double p_k, double p_v, double p_a, double p_time, double p_otherTime, double p_filler) {
        x = p_x;
        y = p_y;
        t = p_t;
        k = p_k;
        v = p_v;
        a = p_a;
        time = p_time;
        otherTime = p_otherTime;
    }

    public ControlPoint forwardTraverse(Vector2d[] p_vals) {
        double n_t = atan2(p_vals[1].getY(), p_vals[1].getX());
        double n_k = (p_vals[1].getX() * p_vals[2].getY() - p_vals[1].getY() * p_vals[2].getX()) / pow(p_vals[1].getX() * p_vals[1].getX() + p_vals[1].getY() * p_vals[1].getY(), 1.5);
        double avgK = (n_k + k) * 0.5;
        double newMaxAccel = MAX_ACCEL - v * v * avgK;
        double dist = sqrt((p_vals[0].getX() - x) * (p_vals[0].getX() - x) + (p_vals[0].getY() - y) * (p_vals[0].getY() - y));
        double radians = 2 * asin(dist * avgK * 0.5);
        double arcLength = radians / avgK;
        double deltaT = (-v + sqrt(v * v + 2 * newMaxAccel * arcLength)) / newMaxAccel;
        double n_time = time + deltaT;
        double n_v = FastMath.min(v + newMaxAccel * deltaT, MAX_VEL * newMaxAccel / MAX_ACCEL);
        double n_a = (n_v - v) / deltaT;
        return new ControlPoint(p_vals[0].getX(), p_vals[0].getY(), n_t, n_k, n_v, n_a, n_time);
    }

    public ControlPoint backwardTraverse(ControlPoint prev) {
        double avgK = (prev.k + k) * 0.5;
        double newMaxAccel = MAX_ACCEL - v * v * avgK;
        double dist = sqrt((prev.x - x) * (prev.x - x) + (prev.y - y) * (prev.y - y));
        double radians = 2 * asin(dist * avgK * 0.5);
        double arcLength = radians / avgK;
        double deltaT = (-v + sqrt(v * v + abs(2 * newMaxAccel * arcLength))) / newMaxAccel;
        double n_v = v + abs(newMaxAccel * deltaT);
        double n_time, o_time;
        if (n_v >= prev.v) {
            n_v = prev.v;
            n_time = prev.time;
            o_time = time - deltaT;
        } else {
            n_time = time - deltaT;
            o_time = prev.time;
        }
        double n_a = (v - n_v) / deltaT;
        return new ControlPoint(prev.x, prev.y, prev.t, prev.k, n_v, n_a, n_time, o_time, 0);
    }

    public double getCentripetalAccel() {
        return v * v * k;
    }
    public void setPrevK(double p_k){
        prevK = p_k;
    }
    public void setNextK(double p_k){
        nextK = p_k;
    }
    public Pose2d getPose() {
        return new Pose2d(x, y, t);
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

    public Vector2d getHeadVec(){
        return new Vector2d(k, (nextK-prevK)*0.5);
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
    public double getKAccel(){return nextK-k;}

    public double getV() {
        return v;
    }

    public double getA() {
        return a;
    }

}
