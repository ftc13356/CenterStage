package org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController;

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
    private final double x, y, t, k, v, a, time;


    public ControlPoint(double p_x, double p_y, double p_dx, double p_dy, double p_ddx, double p_ddy, double p_v, double p_time) {
        x = p_x;
        y=p_y;
        t = atan2(p_dy, p_dx);
        k = (p_dx*p_ddy - p_dy*p_ddx) / pow(p_dx*p_dx+p_dy*p_dy,1.5);
        time = p_time;
        v = p_v;
        a=0;
    }

    public ControlPoint(double p_x, double p_y, double p_t, double p_k, double p_v, double p_a, double p_time) {
        x = p_x;
        y = p_y;
        t = p_t;
        k = p_k;
        v = p_v;
        a = p_a;
        time = p_time;
    }
    public ControlPoint forwardTraverse(double p_x, double p_y, double p_dx, double p_dy, double p_ddx, double p_ddy){
        double n_t = atan2(p_dy, p_dx);
        double n_k = (p_dx*p_ddy - p_dy*p_ddx) / pow(p_dx*p_dx+p_dy*p_dy,1.5);
        double avgK = (n_k + k)*0.5;
        double newMaxAccel =max( MAX_ACCEL - v*v*avgK,1);
        double dist = sqrt((p_x - x) * (p_x - x) + (p_y - y)*(p_y - y));
        double radians = 2 * asin(dist*avgK*0.5);
        double arcLength = radians/avgK;
        double deltaT = (-v + sqrt(v*v+2*newMaxAccel*arcLength))/newMaxAccel;
        double n_time = time + deltaT;
        double n_v = FastMath.min(v + newMaxAccel*deltaT, MAX_VEL*newMaxAccel/MAX_ACCEL);
        double n_a = (n_v- v)/deltaT;
        return new ControlPoint(p_x,p_y,n_t,n_k,n_v,n_a,n_time);
    }
    public double getCentripetalAccel(){
        return v*v*k;
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
