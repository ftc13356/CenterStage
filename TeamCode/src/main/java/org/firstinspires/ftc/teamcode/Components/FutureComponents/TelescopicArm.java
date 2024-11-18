package org.firstinspires.ftc.teamcode.Components.FutureComponents;

import static org.firstinspires.ftc.teamcode.Components.Arm.ArmStates.DROP;
import static org.firstinspires.ftc.teamcode.Components.Arm.ArmStates.HOVER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.Components.Arm;
import org.firstinspires.ftc.teamcode.Components.ExtendoArm.DualPIDController;
import org.firstinspires.ftc.teamcode.Components.Lift;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;

/**
 * Hyun
 * Arm
 */
public class TelescopicArm extends DualPIDController {
    public static double INTAKE_EXTEND_POS = 0;
    public static double INTAKE_PITCH_POS = 0;
    public static double HIGHBUCKET_EXTEND_POS = 0;
    public static double HIGHBUCKET_PITCH_POS = 0;
    public static double LOWBUCKET_EXTEND_POS = 0;
    public static double LOWBUCKET_PITCH_POS = 0;
    public static double HIGHSPECIMEN_EXTEND_POS = 0;
    public static double HIGHSPECIMEN_PITCH_POS = 0;
    public static double LOWSPECIMEN_EXTEND_POS = 0;
    public static double LOWSPECIMEN_PITCH_POS = 0;
    public static double SPECIMENGRAB_EXTEND_POS = 0;
    public static double SPECIMENGRAB_PITCH_POS = 0;
    public static double SAMPLEDROP_EXTEND_POS = 0;
    public static double SAMPLEDROP_PITCH_POS = 0;
    public static double HOVER_EXTEND_POS = 0;
    public static double HOVER_PITCH_POS = 0;
    public static double HANG_EXTEND_POS = 0;
    public static double HANG_PITCH_POS = 0;

    private final double EXTEND_MOTOR_BUFFER = 0;
    private final double PITCH_MOTOR_BUFFER = 0;

    /**
     * init
     */
    public TelescopicArm(){
        super();
    }

    /**
     * possible states of arm
     */
    public enum ArmStates{
        INTAKE(false, INTAKE_EXTEND_POS, INTAKE_PITCH_POS),
        HIGH_BUCKET(false, HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS),
        LOW_BUCKET(false, LOWBUCKET_EXTEND_POS, LOWBUCKET_PITCH_POS),
        HIGH_SPECIMEN(false, HIGHSPECIMEN_EXTEND_POS, HIGHSPECIMEN_PITCH_POS),
        LOW_SPECIMEN(false, LOWSPECIMEN_EXTEND_POS, LOWSPECIMEN_PITCH_POS),
        SPECIMEN_GRAB(false, SPECIMENGRAB_EXTEND_POS, SPECIMENGRAB_PITCH_POS),
        SAMPLE_DROP(false, SAMPLEDROP_EXTEND_POS, SAMPLEDROP_PITCH_POS),
        HOVER(true, HOVER_EXTEND_POS, HOVER_PITCH_POS),
        HANG(false, HANG_EXTEND_POS, HANG_PITCH_POS);

        boolean state;
        double pitchPos;
        double extendPos;

        ArmStates(boolean p_state, double p_extend, double p_pitch){
            state = p_state;
            extendPos = p_extend;
            pitchPos = p_pitch;
        }
        /**
         * sets current state to true
         */
        public void setStateTrue() {
            for(int i=0;i<ArmStates.values().length;i++){
                ArmStates.values()[i].state=false;
            }
            this.state = true;
        }

        public boolean getState(){return this.state;}
    }

    public enum ArmTargetStates{
        INTAKE(false, INTAKE_EXTEND_POS, INTAKE_PITCH_POS),
        HIGH_BUCKET(false, HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS),
        LOW_BUCKET(false, LOWBUCKET_EXTEND_POS, LOWBUCKET_PITCH_POS),
        HIGH_SPECIMEN(false, HIGHSPECIMEN_EXTEND_POS, HIGHSPECIMEN_PITCH_POS),
        LOW_SPECIMEN(false, LOWSPECIMEN_EXTEND_POS, LOWSPECIMEN_PITCH_POS),
        SPECIMEN_GRAB(false, SPECIMENGRAB_EXTEND_POS, SPECIMENGRAB_PITCH_POS),
        SAMPLE_DROP(false, SAMPLEDROP_EXTEND_POS, SAMPLEDROP_PITCH_POS),
        HOVER(true, HOVER_EXTEND_POS, HOVER_PITCH_POS),
        HANG(false, HANG_EXTEND_POS, HANG_PITCH_POS);

        boolean state;
        double pitchPos;
        double extendPos;

        ArmTargetStates(boolean p_state, double p_extend, double p_pitch){
            state = p_state;
            extendPos = p_extend;
            pitchPos = p_pitch;
        }
        /**
         * sets current state to true
         */
        public void setStateTrue() {
            for(int i=0;i<ArmTargetStates.values().length;i++){
                ArmTargetStates.values()[i].state=false;
            }
            this.state = true;
        }

        public boolean getState(){return this.state;}
    }

    public void goTo(TelescopicArm.ArmStates p_state) {
        super.goTo(p_state.extendPos, p_state.pitchPos);
    }

    public void goTo(double p_extend, double p_pitch){
        super.goTo(p_extend,p_pitch);
    }
    /**
     * updates the state machine
     */
    public void update() {
        for (var i : TelescopicArm.ArmStates.values()) {
            if (abs(super.getExtPosition()-i.extendPos) < EXTEND_MOTOR_BUFFER && abs(super.getRotPosition()-i.pitchPos) < PITCH_MOTOR_BUFFER) {
                i.setStateTrue();
                TelescopicArm.ArmTargetStates.values()[i.ordinal()].state = false;
            }
        }
        for (var i : TelescopicArm.ArmTargetStates.values()) {
            if (i.state && abs(super.getExtPosition()-i.extendPos) > EXTEND_MOTOR_BUFFER && abs(super.getRotPosition()-i.pitchPos) > PITCH_MOTOR_BUFFER) {
                goTo(TelescopicArm.ArmStates.values()[i.ordinal()]);
            }
        }
    }
}
