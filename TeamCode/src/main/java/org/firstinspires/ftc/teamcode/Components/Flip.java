package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.Flip.FlipStates.RESET;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
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
    public static double RESET_POS = 0.64;
    public static double RETRACT_POS = 0.55;
    public static double SUBMERSIBLE_POS = 0.3;
    public static double SPECIMEN_POS = .96;
    public static double SPECIMENGRAB_POS = 0.83;
    public static double BUCKET_POS = 0.72, AUTO_GRAH_POS= .63,
            FLIP_TIME = 0.1;

    private final double FLIP_SERVO_BUFFER = 0.05;


    /**
     * init
     */
    public Flip(){
        flip = new RFServo("flipServo",1);
        flip.setLastTime(-100);
        for(int i=0;i<FlipTargetStates.values().length;i++){
            FlipTargetStates.values()[i].state=false;
        }
        if(!isTeleop) {
            flip.setPosition(RESET_POS);
            RESET.setStateTrue();
        }
        flip.setFlipTime(FLIP_TIME);
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
        RETRACT(false, Flip.RETRACT_POS),
        BUCKET(false, BUCKET_POS),
        AUTO_GRAH(false, AUTO_GRAH_POS);
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
        RESET(false, RESET_POS),
        SUBMERSIBLE(false, SUBMERSIBLE_POS),
        SPECIMEN(false, SPECIMEN_POS),
        SPECIMEN_GRAB(false, SPECIMENGRAB_POS),
        RETRACT(false, RETRACT_POS),
        BUCKET(false, BUCKET_POS),
        AUTO_GRAH(false, AUTO_GRAH_POS);

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
                i.state = true;
                Flip.FlipTargetStates.values()[i.ordinal()].state = false;
            }
            else{
                i.state = false;
            }
        }
        for (var i : Flip.FlipTargetStates.values()) {
            if (i.state && abs(flip.getPosition()-i.position) > FLIP_SERVO_BUFFER) {
//                flipTo(Flip.FlipStates.values()[i.ordinal()]);
            }
        }
        flip.update();
        packet.put("FLIPRESET", RESET.getState());
    }
}
