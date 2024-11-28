package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.Twist.TwistStates.PARALLEL;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

/**
 * Hyun
 * Claw
 */
@Config
public class Claw {
    RFServo claw;

    public static double OPEN_POS = 0.6;
    public static double CLOSED_POS = 0.38;

    public static double FLIP_TIME = 0.5;
    private final double CLAW_SERVO_BUFFER = 0.05;
    /**
     * init
     */
    public Claw(){
        claw = new RFServo("clawServo",1);
        if(!isTeleop) {
            claw.setPosition(CLOSED_POS);
            ClawStates.CLOSED.setStateTrue();
        }

        claw.setLastTime(-100);
    }

    /**
     * possible states of claw
     */
    public enum ClawStates{
        OPEN(true, OPEN_POS),
        CLOSED(false, CLOSED_POS);
        boolean state;
        double position;
        ClawStates(boolean p_state, double p_position){
            position = p_position;
            state = p_state;
        }
        /**
         * sets current state to true
         */
        public void setStateTrue() {
            for(int i=0;i<ClawStates.values().length;i++){
                ClawStates.values()[i].state=false;
            }
            this.state = true;
        }
        public boolean getState(){return this.state;}
    }

    public enum ClawTargetStates{
        OPEN(true, OPEN_POS),
        CLOSED(false, CLOSED_POS);
        boolean state;
        double position;
        ClawTargetStates(boolean p_state, double p_position){
            position = p_position;
            state = p_state;
        }
        /**
         * sets current state to true
         */
        public void setStateTrue() {
            for(int i=0;i<ClawTargetStates.values().length;i++){
                ClawTargetStates.values()[i].state=false;
            }
            this.state = true;
        }
        public boolean getState(){return this.state;}
    }

    public void goTo(double p_position){
        claw.setPosition(p_position);
    }

    public void goTo(Claw.ClawStates p_state){
        claw.setPosition(p_state.position);
    }

    /**
     * updates the state machine
     */
    public void update() {
        for (var i : Claw.ClawStates.values()) {
            if (abs(claw.getPosition() - i.position) < CLAW_SERVO_BUFFER && time - claw.getLastTime() > FLIP_TIME) {
                i.setStateTrue();
                Claw.ClawTargetStates.values()[i.ordinal()].state = false;
            }
        }
        for (var i : Claw.ClawTargetStates.values()) {
            if (i.state && abs(claw.getPosition() - i.position) > CLAW_SERVO_BUFFER) {
                goTo(Claw.ClawStates.values()[i.ordinal()]);
            }
        }
    }
}
