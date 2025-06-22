package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

@Config
public class BigLight {
    RFServo bigLight;

    public static double ON_POS = 1;
    public static double OFF_POS = 0;

    public static double LIGHT_TIME = 0.4;
    private final double LIGHT_BUFFER = 0.05;
    /**
     * init
     */
    public BigLight(){
        bigLight = new RFServo("bigLight",1);
        bigLight.setLastTime(-100);
        for(int i = 0; i< BigLightTargetStates.values().length; i++){
            BigLightTargetStates.values()[i].state=false;
        }
//        if(!isTeleop) {
            bigLight.setPosition(ON_POS); // change i guess
            BigLightStates.ON.setStateTrue();
//        }
        bigLight.setLastTime(-100);
        bigLight.setFlipTime(LIGHT_TIME);
    }

    /**
     * possible states of light
     */
    public enum BigLightStates {
        ON(true, ON_POS),
        OFF(false, OFF_POS);
        boolean state;
        double position;
        BigLightStates(boolean p_state, double p_position){
            position = p_position;
            state = p_state;
        }
        /**
         * sets current state to true
         */
        public void setStateTrue() {
            for(int i = 0; i< BigLightStates.values().length; i++){
                BigLightStates.values()[i].state=false;
            }
            this.state = true;
        }
        public boolean getState(){return this.state;}
    }

    public enum BigLightTargetStates {
        ON(true, ON_POS),
        OFF(false, OFF_POS);
        boolean state;
        double position;
        BigLightTargetStates(boolean p_state, double p_position){
            position = p_position;
            state = p_state;
        }
        /**
         * sets current state to true
         */
        public void setStateTrue() {
            for(int i = 0; i< BigLightTargetStates.values().length; i++){
                BigLightTargetStates.values()[i].state=false;
            }
            this.state = true;
        }
        public boolean getState(){return this.state;}
    }

    public void goTo(double p_position){
        bigLight.setPosition(p_position);
    }

    public void goTo(BigLight.BigLightStates p_state){
        BigLight.BigLightTargetStates.values()[p_state.ordinal()].state = true;
        bigLight.setPosition(p_state.position);
    }

    /**
     * updates the state machine
     */
    public void update() {
        for (var i : BigLightStates.values()) {
            if (abs(bigLight.getPosition() - i.position) < LIGHT_BUFFER && time - bigLight.getLastTime() > LIGHT_TIME) {
                i.setStateTrue();
                BigLight.BigLightTargetStates.values()[i.ordinal()].state = false;
            }

        }
        bigLight.update();
        if(BigLightStates.ON.getState())
            packet.put("BIGLIGHT_STATE", "ON");
        else
            packet.put("BIGLIGHT_STATE", "OFF");
    }
}
