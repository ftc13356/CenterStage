package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.Flip.FlipStates.RESET;
import static org.firstinspires.ftc.teamcode.Components.Twist.TwistStates.PARALLEL;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

/**
 * Hyun
 * Flip
 */
@Config
public class Flip {
    RFServo flip;
    public static double RESET_POS = 1;
    public static double SUBMERSIBLE_POS = 0.6;
    public static double SPECIMEN_POS = 1;
    public static double SPECIMENGRAB_POS = 1;
    public static double BASKET_POS = 1, FLIP_TIME = 0.5;

    private final double FLIP_SERVO_BUFFER = 0.05;


    /**
     * init
     */
    public Flip(){
        flip = new RFServo("flipServo",1);
        if(!isTeleop) {
            flip.setPosition(RESET_POS);
            RESET.setStateTrue();
        }
        flip.setLastTime(-100);
    }

    /**
     * possible states of flip
     */
    public enum FlipStates{
        RESET(true, RESET_POS),
        SUBMERSIBLE(false, SUBMERSIBLE_POS),
        SPECIMEN(false, SPECIMEN_POS),
        SPECIMEN_GRAB(false, SPECIMENGRAB_POS),
        BASKET(false, BASKET_POS);
        boolean state;
        double position;
        FlipStates(boolean p_state, double p_position){
            state = p_state;
            position = p_position;
        }
        /**
         * sets current state to true
         */
        public void setStateTrue() {
            for(int i=0;i<FlipStates.values().length;i++){
                FlipStates.values()[i].state=false;
            }
            this.state = true;
        }
        public boolean getState(){return this.state;}
    }

    public enum FlipTargetStates{
        RESET(true, RESET_POS),
        SUBMERSIBLE(false, SUBMERSIBLE_POS),
        SPECIMEN(false, SPECIMEN_POS),
        SPECIMEN_GRAB(false, SPECIMENGRAB_POS),
        BASKET(false, BASKET_POS);
        boolean state;
        double position;
        FlipTargetStates(boolean p_state, double p_position){
            state = p_state;
            position = p_position;
        }
        /**
         * sets current state to true
         */
        public void setStateTrue() {
            for(int i=0;i<FlipTargetStates.values().length;i++){
                FlipTargetStates.values()[i].state=false;
            }
            this.state = true;
        }
        public boolean getState(){return this.state;}
    }

    public void flipTo(Flip.FlipStates p_state){
        flip.setPosition(p_state.position);
    }

    public void flipTo(double p_position){
        flip.setPosition(p_position);
    }

    /**
     * updates the state machine
     */
    public void update() {
        for (var i : Flip.FlipStates.values()) {
            if (abs(flip.getPosition() - i.position) < FLIP_SERVO_BUFFER && time - flip.getLastTime() > FLIP_TIME) {
                i.setStateTrue();
                Flip.FlipTargetStates.values()[i.ordinal()].state = false;
            }
        }
        for (var i : Flip.FlipTargetStates.values()) {
            if (i.state && abs(flip.getPosition()-i.position) > FLIP_SERVO_BUFFER) {
                flipTo(Flip.FlipStates.values()[i.ordinal()]);
            }
        }
    }
}
