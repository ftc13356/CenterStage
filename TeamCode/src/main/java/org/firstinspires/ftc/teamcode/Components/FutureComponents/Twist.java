package org.firstinspires.ftc.teamcode.Components.FutureComponents;

import static java.lang.Math.abs;

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
    public static double PERPENDICULAR_POS = 0;
    public static double GRAB_POS = 0;
    private final double TWIST_SERVO_BUFFER = 0;

    /**
     * init
     */
    public Twist(){
        twist = new RFServo("twistServo",1);
    }

    /**
     * possible states of twist
     */
    public enum TwistStates{
        PARALLEL(true, PARALLEL_POS),
        PERPENDICULAR(false, PERPENDICULAR_POS),
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
        PARALLEL(true, PARALLEL_POS),
        PERPENDICULAR(false, PERPENDICULAR_POS),
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

    /**
     * updates the state machine
     */
    public void update() {
        for (var i : Twist.TwistStates.values()) {
            if (abs(twist.getPosition()-i.position) < TWIST_SERVO_BUFFER) {
                i.setStateTrue();
                Twist.TwistStates.values()[i.ordinal()].state = false;
            }
        }
        for (var i : Twist.TwistTargetStates.values()) {
            if (i.state && abs(twist.getPosition()-i.position) > TWIST_SERVO_BUFFER) {
                twistTo(Twist.TwistStates.values()[i.ordinal()]);
            }
        }
    }
}
