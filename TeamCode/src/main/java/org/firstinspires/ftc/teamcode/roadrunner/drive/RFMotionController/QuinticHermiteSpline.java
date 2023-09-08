package org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController;


import static org.apache.commons.math3.util.FastMath.pow;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import java.util.ArrayList;

public class QuinticHermiteSpline {
    private ArrayList<Vector2d> coeffs;

    public QuinticHermiteSpline(Vector2d p_startPos, Vector2d p_startVel, Vector2d p_startAccel, Vector2d p_endPos, Vector2d p_endVel, Vector2d p_endAccel) {
        coeffs.add(p_startPos);
        coeffs.add(p_startVel);
        coeffs.add(p_startAccel.times(0.5));
        coeffs.add(p_startPos.times(-10).plus(p_startVel.times(-6)).plus(p_startAccel.times(-1.5))
                .plus(p_endAccel.times(-1)).plus(p_endVel.times(-4)).plus(p_endPos).times(-10));
        coeffs.add(p_startPos.times(15).plus(p_startVel.times(8)).plus(p_startAccel.times(1.5))
                .plus(p_endAccel.times(-1)).plus(p_endVel.times(7)).plus(p_endPos.times(-15)));
        coeffs.add(p_startPos.times(-6).plus(p_startVel.times(-3)));
    }

    //return x,y,dx,dy,ddx,ddy
    public Vector2d[] valsAt(double p_t){
        Vector2d[] vals = {new Vector2d(0,0),new Vector2d(0,0), new Vector2d(0,0)};
        for(int i=0;i<coeffs.size();i++){
            vals[0]=vals[0].plus(coeffs.get(i).times(pow(p_t,i)));
        }
        for(int i=1;i<coeffs.size();i++){
            vals[1]=vals[1].plus(coeffs.get(i).times(pow(p_t,i-1)).times(i));
        }
        for(int i=2;i<coeffs.size();i++){
            vals[1]=vals[1].plus(coeffs.get(i).times(pow(p_t,i-2)).times(i*(i-1)));
        }
        return vals;
    }
    public Pose2d poseAt(double p_t) {
        Vector2d pose = new Vector2d(0, 0);
        //eval func
        for (int i = 0; i < coeffs.size(); i++) {
            pose = pose.plus(coeffs.get(i).times(pow(p_t, i)));
        }
        Vector2d velo = new Vector2d(0, 0);
        //eval first deriv
        for (int i = 1; i < coeffs.size(); i++) {
            velo = velo.plus(coeffs.get(i).times(i*pow(p_t, i - 1)));
        }
        return new Pose2d(pose, velo.angle());
    }
}
