package org.firstinspires.ftc.teamcode.roadrunner.drive.RFMotionController;


import static org.apache.commons.math3.util.FastMath.pow;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.dashboard;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.checkerframework.checker.units.qual.A;

import java.util.ArrayList;

public class QuinticHermiteSpline {
    private ArrayList<Vector2d> coeffs;

    public void initQuinticHermiteSpline(Vector2d p_startPos, Vector2d p_startVel, Vector2d p_startAccel, Vector2d p_endPos, Vector2d p_endVel, Vector2d p_endAccel) {
        coeffs = new ArrayList<>();
        coeffs.add(p_startPos);
        coeffs.add(p_startVel);
        coeffs.add(p_startAccel.times(0.5));
        coeffs.add(p_startPos.times(-10).plus(p_startVel.times(-6)).plus(p_startAccel.times(-1.5))
                .plus(p_endAccel.times(-1)).plus(p_endVel.times(-4)).plus(p_endPos).times(-10));
        coeffs.add(p_startPos.times(15).plus(p_startVel.times(8)).plus(p_startAccel.times(1.5))
                .plus(p_endAccel.times(-1)).plus(p_endVel.times(7)).plus(p_endPos.times(-15)));
        coeffs.add(p_startPos.times(-6).plus(p_startVel.times(-3)).plus(p_startAccel.times(-0.5))
                .plus(p_endAccel.times(0.5)).plus(p_endVel.times(-3)).plus(p_endPos.times(6)));
        for(int i=0;i<coeffs.size(); i++){
            packet.put("coeffs"+i, coeffs.get(i));
        }
    }
    public QuinticHermiteSpline(Vector2d[] p_coeffs) {
        initQuinticHermiteSpline(p_coeffs[0],p_coeffs[1],p_coeffs[2],p_coeffs[3],p_coeffs[4],p_coeffs[5]);
    }
    public QuinticHermiteSpline(Vector2d p_startPos, Vector2d p_startVel, Vector2d p_startAccel, Vector2d p_endPos, Vector2d p_endVel, Vector2d p_endAccel) {
        coeffs = new ArrayList<>();
        coeffs.add(p_startPos);
        coeffs.add(p_startVel);
        coeffs.add(p_startAccel.times(0.5));
        coeffs.add(p_startPos.times(-10).plus(p_startVel.times(-6)).plus(p_startAccel.times(-1.5))
                .plus(p_endAccel.times(-1)).plus(p_endVel.times(-4)).plus(p_endPos).times(-10));
        coeffs.add(p_startPos.times(15).plus(p_startVel.times(8)).plus(p_startAccel.times(1.5))
                .plus(p_endAccel.times(-1)).plus(p_endVel.times(7)).plus(p_endPos.times(-15)));
        coeffs.add(p_startPos.times(-6).plus(p_startVel.times(-3)).plus(p_startAccel.times(-0.5))
                .plus(p_endAccel.times(0.5)).plus(p_endVel.times(-3)).plus(p_endPos.times(6)));
        for(int i=0;i<coeffs.size(); i++){
            packet.put("coeffs"+i, coeffs.get(i));
        }
    }

    /**
     *
     * @param p_t
     * @return x, dx, ddx
     */
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
        packet.put("vals", vals[0]);
        dashboard.sendTelemetryPacket(packet);
        return vals;
    }
}
