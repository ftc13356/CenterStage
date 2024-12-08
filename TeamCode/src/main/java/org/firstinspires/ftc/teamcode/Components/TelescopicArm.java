package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static java.lang.Math.PI;
import static java.lang.Math.abs;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.min;
import static java.lang.Math.sin;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.DualPIDController;

/**
 * Hyun
 * Arm
 */
@Config
public class TelescopicArm extends DualPIDController {
    public static double INTAKE_EXTEND_POS = 5;
    public static double INTAKE_PITCH_POS = 5;
    public static double HIGHBUCKET_EXTEND_POS = 30;
    public static double HIGHBUCKET_PITCH_POS = 96;
    public static double LOWBUCKET_EXTEND_POS = 11;
    public static double LOWBUCKET_PITCH_POS = 100;
    public static double HIGHSPECIMEN_EXTEND_POS = 17.5;
    public static double HIGHSPECIMEN_PITCH_POS = 41;
    public static double LOWSPECIMEN_EXTEND_POS = 10;
    public static double LOWSPECIMEN_PITCH_POS = 25;
    public static double SPECIMENGRAB_EXTEND_POS = 0;
    public static double SPECIMENGRAB_PITCH_POS = 160;
    public static double HOVER_EXTEND_POS = 5;
    public static double HOVER_PITCH_POS = 23;
    public static double HANG_EXTEND_POS = 5;
    public static double HANG_PITCH_POS = 70, RETRACTED_EXTEND__POS = 0, RETRACTED_PITCH_POS = 0, MANUAL_EXT_SPEED = 0.5, MANUAL_ROT_SPEED = 0.5;

    private final double EXTEND_MOTOR_BUFFER = 5;
    private final double PITCH_MOTOR_BUFFER = 5;

    double lastManualTime = -100;

    /**
     * init
     */
    public TelescopicArm() {
        super();
        lastManualTime = -100;
        for (int i = 0; i < ArmTargetStates.values().length; i++) {
            ArmTargetStates.values()[i].state = false;
        }
        if(!isTeleop){
            ArmStates.RETRACTED.setStateTrue();
        }
        ArmStates.HIGH_SPECIMEN.pitchPos = HIGHSPECIMEN_PITCH_POS;
        ArmStates.HIGH_SPECIMEN.extendPos = HIGHSPECIMEN_EXTEND_POS;
        ArmTargetStates.HIGH_SPECIMEN.pitchPos = HIGHSPECIMEN_PITCH_POS;
        ArmTargetStates.HIGH_SPECIMEN.extendPos = HIGHSPECIMEN_EXTEND_POS;
    }

    /**
     * possible states of arm
     */
    public enum ArmStates {
        INTAKE(false, INTAKE_EXTEND_POS, INTAKE_PITCH_POS),
        HOVER(false, HOVER_EXTEND_POS, HOVER_PITCH_POS),
        HIGH_BUCKET(false, HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS),
        LOW_BUCKET(false, LOWBUCKET_EXTEND_POS, LOWBUCKET_PITCH_POS),
        HIGH_SPECIMEN(false, HIGHSPECIMEN_EXTEND_POS, HIGHSPECIMEN_PITCH_POS),
        LOW_SPECIMEN(false, LOWSPECIMEN_EXTEND_POS, LOWSPECIMEN_PITCH_POS),
        SPECIMEN_GRAB(false, SPECIMENGRAB_EXTEND_POS, SPECIMENGRAB_PITCH_POS),
        RETRACTED(true, RETRACTED_EXTEND__POS, RETRACTED_PITCH_POS),

        HANG(false, HANG_EXTEND_POS, HANG_PITCH_POS);

        boolean state;
        double pitchPos;
        double extendPos;

        ArmStates(boolean p_state, double p_extend, double p_pitch) {
            state = p_state;
            extendPos = p_extend;
            pitchPos = p_pitch;
        }

        /**
         * sets current state to true
         */
        public void setStateTrue() {
            for (int i = 0; i < ArmStates.values().length; i++) {
                ArmStates.values()[i].state = false;
            }
            this.state = true;
        }

        public boolean getState() {
            return this.state;
        }
    }

    public enum ArmTargetStates {
        INTAKE(false, INTAKE_EXTEND_POS, INTAKE_PITCH_POS),
        HOVER(false, HOVER_EXTEND_POS, HOVER_PITCH_POS),
        HIGH_BUCKET(false, HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS),
        LOW_BUCKET(false, LOWBUCKET_EXTEND_POS, LOWBUCKET_PITCH_POS),
        HIGH_SPECIMEN(false, HIGHSPECIMEN_EXTEND_POS, HIGHSPECIMEN_PITCH_POS),
        LOW_SPECIMEN(false, LOWSPECIMEN_EXTEND_POS, LOWSPECIMEN_PITCH_POS),
        SPECIMEN_GRAB(false, SPECIMENGRAB_EXTEND_POS, SPECIMENGRAB_PITCH_POS),
        RETRACTED(false, RETRACTED_EXTEND__POS, RETRACTED_PITCH_POS),
        HANG(false, HANG_EXTEND_POS, HANG_PITCH_POS);

        boolean state;
        double pitchPos;
        double extendPos;

        ArmTargetStates(boolean p_state, double p_extend, double p_pitch) {
            state = p_state;
            extendPos = p_extend;
            pitchPos = p_pitch;
        }

        /**
         * sets current state to true
         */
        public void setStateTrue() {
            for (int i = 0; i < ArmTargetStates.values().length; i++) {
                ArmTargetStates.values()[i].state = false;
            }
            this.state = true;
        }

        public boolean getState() {
            return this.state;
        }
    }

    public void manualGoTo(double p_extend, double p_pitch) {
        if(!ArmStates.HOVER.getState() && p_pitch<0.1) {
            super.goTo(super.getTargetExt()+p_extend*MANUAL_EXT_SPEED, super.getTargetRot()+p_pitch*MANUAL_ROT_SPEED);
            for (var i : TelescopicArm.ArmTargetStates.values()) {
                i.state = false;
            }
        } else{
            super.goTo(super.getTargetExt()+p_extend*MANUAL_EXT_SPEED, Math.atan2(5, super.getTargetExt()+p_extend*MANUAL_EXT_SPEED+8)*180/PI);
        }
    }
    public void lowerToIntake(){
        super.goTo((super.getExt()+7)*cos((super.getRot())*PI/180)-7,0);
        ArmStates.INTAKE.setStateTrue();
    }

    public void goTo(TelescopicArm.ArmStates p_state) {
        if(p_state == ArmStates.RETRACTED){
            if(Claw.ClawStates.OPEN.getState()){
                super.goTo(p_state.extendPos, 0,0, getRot());
            }
            else{
                super.goTo(p_state.extendPos, 100, 0, getRot());
            }
            return;
        }
        if(ArmStates.RETRACTED.getState()){
            double middle = 0;
            if(abs(p_state.pitchPos-getRot())>40){
                middle = 0;
            }
            else{
                middle = min(p_state.extendPos, getExt());
            }
            super.goTo(p_state.extendPos,p_state.pitchPos,middle, 0.5*(p_state.pitchPos+getRot()));
            return;
        }
        if (ArmStates.INTAKE.getState() || ArmStates.LOW_SPECIMEN.getState() || ArmStates.HOVER.getState()) {
            if (p_state == ArmStates.HOVER || p_state == ArmStates.INTAKE || p_state == ArmStates.LOW_SPECIMEN) {
                super.goTo(p_state.extendPos, p_state.pitchPos);
            } else {
                double middle = 0;
                if(abs(p_state.pitchPos-getRot())>50){
                    middle = 0;
                }
                else{
                    middle = min(p_state.extendPos, getExt());
                }
                super.goTo(p_state.extendPos, p_state.pitchPos, middle, getRot());
            }
        } else if (ArmStates.HIGH_SPECIMEN.getState()) {
            double middle = 0;
            super.goTo(p_state.extendPos, p_state.pitchPos, middle, ArmStates.HIGH_SPECIMEN.pitchPos - 5);
        } else if (ArmStates.HIGH_BUCKET.getState()) {
            if (p_state == ArmStates.LOW_BUCKET || p_state == ArmStates.SPECIMEN_GRAB) {
                super.goTo(p_state.extendPos, p_state.pitchPos, p_state.extendPos, ArmStates.HIGH_BUCKET.pitchPos);
            } else {
                double middle = 0;
                if(abs(p_state.pitchPos-getRot())>40){
                    middle = 0;
                }
                else{
                    middle = min(p_state.extendPos, getExt());
                }
                super.goTo(p_state.extendPos, p_state.pitchPos, middle, p_state.pitchPos);
            }
        } else if (ArmStates.LOW_BUCKET.getState()) {
            if (p_state == ArmStates.HIGH_BUCKET || p_state == ArmStates.SPECIMEN_GRAB) {
                super.goTo(p_state.extendPos, p_state.pitchPos, ArmStates.LOW_BUCKET.extendPos, p_state.pitchPos);
            } else {
                super.goTo(p_state.extendPos, p_state.pitchPos);
            }
        } else if (ArmStates.SPECIMEN_GRAB.getState()) {
            super.goTo(p_state.extendPos, p_state.pitchPos, 0, p_state.pitchPos);
        } else {
            super.goTo(p_state.extendPos, p_state.pitchPos);
        }
    }

    public void goTo(double p_extend, double p_pitch) {
        super.goTo(p_extend, p_pitch);
    }
    public double getVel(){
        return super.getVel();
    }

    public void goToResetManual(double p_extend, double p_pitch){
        lastManualTime = -100;
        goTo(p_extend,p_pitch);
    }
    /**
     * updates the state machine
     * TODO: ZEROING SYSTEM
     */
    public void update() {
        boolean targeted = false;
        if (time - lastManualTime > .5) {
            for (var i : TelescopicArm.ArmStates.values()) {
                if (abs(super.getExt() - i.extendPos) < EXTEND_MOTOR_BUFFER && abs(super.getRot() - i.pitchPos) < PITCH_MOTOR_BUFFER) {
                    i.setStateTrue();
                    TelescopicArm.ArmTargetStates.values()[i.ordinal()].state = false;
                }
                else if(i == ArmStates.RETRACTED){
                    if(abs(super.getExt())<3){
                        i.state = true;
                        TelescopicArm.ArmTargetStates.values()[i.ordinal()].state = false;
                    }
                }
                else if(i == ArmStates.INTAKE || i == ArmStates.HOVER){
                    if(abs(super.getRot()-i.pitchPos)<PITCH_MOTOR_BUFFER+3  ){
                        i.state  = true;
                        TelescopicArm.ArmTargetStates.values()[i.ordinal()].state = false;
                    }

                }
            }
            if(abs(getRot()-HIGHSPECIMEN_PITCH_POS)<5){
                ArmStates.HIGH_SPECIMEN.state = true;
                TelescopicArm.ArmTargetStates.values()[ArmStates.HIGH_SPECIMEN.ordinal()].state = false;

            }
            if(abs(5 - (getExt()+8)*sin((getRot())*PI/180))<2.5 && getRot()<90){
                ArmStates.HOVER.state = true;
                TelescopicArm.ArmTargetStates.values()[ArmStates.HOVER.ordinal()].state = false;
            }
            else{
                ArmStates.HOVER.state = false;
            }
            for (var i : TelescopicArm.ArmTargetStates.values()) {
                if (i.state && abs(super.getExt() - i.extendPos) > EXTEND_MOTOR_BUFFER && abs(super.getRot() - i.pitchPos) > PITCH_MOTOR_BUFFER && (isMid() && abs(getExt()-getTargetExt())<3)&&abs(getRot()-getTargetRot())<10) {
                    goTo(TelescopicArm.ArmStates.values()[i.ordinal()]);
                    targeted = true;
                }
            }

        }
        if (!targeted && isMid())
            goTo(getTargetExt(), getTargetRot());
        else if(!isMid())
            goTo(getTrueTargExt(), getTrueTargRot(), getMiddle(), getMiddleRot());
        packet.put("targExt", super.getTargetExt());
        packet.put("targRot", super.getTargetRot());
        packet.put("targMid", super.getMiddle());
        packet.put("targMidRot", super.getMiddleRot());
        packet.put("curExt", super.getExt());
        packet.put("curRot", super.getRot());
        packet.put("horiExt", super.getExt()*cos(getRot()*PI/180));
        packet.put("targVertExt",(getTargetExt()+8)*sin(getTargetRot()*PI/180));
        packet.put("curVertExt",(getExt()+8)*sin((getRot())*PI/180));
        packet.put("diff",abs(5-(getTargetExt()+10)*sin(getRot()*PI/180)));
        packet.put("boolean", abs(5-(getTargetExt()+10)*sin(getRot()*PI/180))>4);
        packet.put("targeted", targeted);
        packet.put("isMid", isMid());
        packet.put("HOVER", ArmStates.HOVER.getState());
        packet.put("hoverbuffa",abs(super.getRot()-HOVER_PITCH_POS));

    }

}
