package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.Twist.TwistStates.PARALLEL;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
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

    public static double OPEN_POS = .48;
    public static double OPEN_POS_TELE = .39;
    public static double CLOSED_POS = 0.00;

    public static double FLIP_TIME = 0.2;
    private final double CLAW_SERVO_BUFFER = 0.05;
    /**
     * init
     */
    public Claw(){
        claw = new RFServo("clawServo",1);
        claw.setLastTime(-100);
        for(int i=0;i<ClawTargetStates.values().length;i++){
            ClawTargetStates.values()[i].state=false;
        }
        if(!isTeleop) {
//            ClawStates.OPEN.position=OPEN_POS;
            claw.setPosition(CLOSED_POS);
            ClawStates.CLOSED.setStateTrue();
        } else {
//            ClawStates.OPEN.position=OPEN_POS_TELE;
        }
        claw.setLastTime(-100);
        claw.setFlipTime(FLIP_TIME);
    }

    /**
     * possible states of claw
     */
    public enum ClawStates{
        GIGA_OPEN(false, OPEN_POS),
        OPEN(true, OPEN_POS_TELE),
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
        GIGA_OPEN(false, OPEN_POS),
        OPEN(false, OPEN_POS_TELE),
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
        Claw.ClawTargetStates.values()[p_state.ordinal()].state = true;
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
        claw.update();
//        for (var i : Claw.ClawTargetStates.values()) {
//            if (i.state && abs(claw.getPosition() - i.position) > CLAW_SERVO_BUFFER) {
//                goTo(Claw.ClawStates.values()[i.ordinal()]);
//            }
//        }
//        packet.put("claw open", ClawStates.OPEN.getState());
    }
}
