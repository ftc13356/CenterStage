package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Components.Twist.TwistStates.PARALLEL;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static java.lang.Math.abs;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;

@Config
public class Hardstop {
    RFServo hardstop;

    public static double STOP_POS = 0;
    public static double GO_POS  = 1;

    public static double FLIP_TIME = 0.4;
    private final double STOP_SERVO_BUFFER = 0.05;
    /**
     * init
     */
    public Hardstop(){
        hardstop = new RFServo("hardstopServo",1);
        hardstop.setLastTime(-100);
        for(int i=0;i<HardstopTargetStates.values().length;i++){
            HardstopTargetStates.values()[i].state=false;
        }
        if(!isTeleop) {
            hardstop.setPosition(GO_POS);
            HardstopStates.GO.setStateTrue();
        } else {
//            ClawStates.OPEN.position=OPEN_POS_TELE;
        }
        hardstop.setLastTime(-100);
        hardstop.setFlipTime(FLIP_TIME);
    }

    /**
     * possible states of hardstop
     */
    public enum HardstopStates{
        GO(true, GO_POS),
        STOP(false, STOP_POS);
        boolean state;
        double position;
        HardstopStates(boolean p_state, double p_position){
            position = p_position;
            state = p_state;
        }
        /**
         * sets current state to true
         */
        public void setStateTrue() {
            for(int i=0;i<HardstopStates.values().length;i++){
                HardstopStates.values()[i].state=false;
            }
            this.state = true;
        }

        public boolean getState(){return this.state;}
    }

    public enum HardstopTargetStates{
        GO(true, GO_POS),
        STOP(false, STOP_POS);
        boolean state;
        double position;
        HardstopTargetStates(boolean p_state, double p_position){
            position = p_position;
            state = p_state;
        }
        /**
         * sets current state to true
         */
        public void setStateTrue() {
            for(int i=0;i<HardstopTargetStates.values().length;i++){
                HardstopTargetStates.values()[i].state=false;
            }
            this.state = true;
        }
        public boolean getState(){return this.state;}
    }

    public void goTo(double p_position){
        hardstop.setPosition(p_position);
    }

    public void goTo(Hardstop.HardstopStates p_state){
        Hardstop.HardstopTargetStates.values()[p_state.ordinal()].state = true;
        hardstop.setPosition(p_state.position);
    }

    /**
     * updates the state machine
     */
    public void update() {
        for (var i : HardstopStates.values()) {
            if (abs(hardstop.getPosition() - i.position) < STOP_SERVO_BUFFER && time - hardstop.getLastTime() > FLIP_TIME) {
                i.setStateTrue();
                Hardstop.HardstopTargetStates.values()[i.ordinal()].state = false;
            }

        }
        hardstop.update();
        if(HardstopStates.GO.getState())
            packet.put("HARDSTOP_STATE", "GO");
        else
            packet.put("HARDSTOP_STATE", "STOP");
    }
}
