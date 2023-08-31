package org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController;

import static org.apache.commons.math3.util.FastMath.cos;
import static org.apache.commons.math3.util.FastMath.sin;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;

public class ControlPoint {
    private final double x, y, t, k, v, a;

    public ControlPoint(Pose2d pose2d, double k, double v, double a) {
        x = pose2d.getX();
        y = pose2d.component2();
        t = pose2d.component3();
        this.k = k;
        this.v = v;
        this.a = a;
    }

    public ControlPoint(double x, double y, double t, double k, double v, double a) {
        this.x = x;
        this.y = y;
        this.t = t;
        this.k = k;
        this.v = v;
        this.a = a;
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
        return new Vector2d(a * cos(t), a * sin(t));
    }

    public double getX() {
        return x;
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

    public double getV() {
        return v;
    }

    public double getA() {
        return a;
    }

}
