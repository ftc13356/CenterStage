package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.Twist.TwistStates.PARALLEL;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static java.lang.Math.abs;
import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

/**
 * Hyun
 * Twist
 */
@Config
public class Twist {
    RFServo twist;
    public static double PARALLEL_POS = 0;
    public static double SPECIMEN_POS = 1;
    public static double GRAB_POS = 0, FLIP_TIME = 0.2;
    private final double TWIST_SERVO_BUFFER = 0.05;

    /**
     * init
     */
    public Twist(){
        twist = new RFServo("twistServo",1);
        twist.setLastTime(-100);
        for(int i=0;i<TwistTargetStates.values().length;i++){
            TwistTargetStates.values()[i].state=false;
        }
        if(!isTeleop) {
            twist.setPosition(PARALLEL_POS);
            PARALLEL.setStateTrue();
        }
    }

    /**
     * possible states of twist
     */
    public enum TwistStates{
        PARALLEL(true, PARALLEL_POS),
        PERPENDICULAR(false, SPECIMEN_POS),
        GRAB(false, GRAB_POS);
        boolean state;
        double position;
        TwistStates(boolean p_state, double p_position){
            state = p_state;
            position = p_position;
        }
        /**
         * sets current state to true
         */
        public void setStateTrue() {
            for(int i=0;i<TwistStates.values().length;i++){
                TwistStates.values()[i].state=false;
            }
            this.state = true;
        }
        public boolean getState(){return this.state;}
    }

    public enum TwistTargetStates{
        PARALLEL(false, PARALLEL_POS),
        PERPENDICULAR(false, SPECIMEN_POS),
        GRAB(false, GRAB_POS);
        boolean state;
        double position;
        TwistTargetStates(boolean p_state, double p_position){
            state = p_state;
            position = p_position;
        }
        /**
         * sets current state to true
         */
        public void setStateTrue() {
            for(int i=0;i<TwistTargetStates.values().length;i++){
                TwistTargetStates.values()[i].state=false;
            }
            this.state = true;
        }
        public boolean getState(){return this.state;}
    }

    public void twistTo(Twist.TwistStates p_state){
        twist.setPosition(p_state.position);
    }

    public void twistTo(double p_pos){
        twist.setPosition(p_pos);
    }

    public void twistToAng(double p_ang){
        if(p_ang<0){
            if(p_ang>-5){
                p_ang=-180;
            }
            p_ang+=180;
        }
        p_ang = min(1,p_ang/150);
        twist.setPosition(p_ang);
    }

    /**
     * updates the state machine
     */
    public void update() {
        for (var i : Twist.TwistStates.values()) {
            if (abs(twist.getPosition()-i.position) < TWIST_SERVO_BUFFER && time - twist.getLastTime() > FLIP_TIME) {
                i.state = true;
            }
            else{
                i.state = false;
            }
        }
        for (var i : Twist.TwistTargetStates.values()) {
            if (i.state && abs(twist.getPosition()-i.position) > TWIST_SERVO_BUFFER) {
                twistTo(Twist.TwistStates.values()[i.ordinal()]);
            }
        }
    }
}
