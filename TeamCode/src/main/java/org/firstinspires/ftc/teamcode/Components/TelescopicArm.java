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
    public static double INTAKE_EXTEND_POS = 4;
    public static double INTAKE_PITCH_POS = 3;
    public static double HIGHBUCKET_EXTEND_POS = 29;
    public static double HIGHBUCKET_PITCH_POS = 102;
    public static double LOWBUCKET_EXTEND_POS = 12;
    public static double LOWBUCKET_PITCH_POS = 108;
    public static double HIGHSPECIMEN_EXTEND_POS = 16.5;
//    public static double HIGHSPECIMEN_PITCH_POS = 40.5;
public static double HIGHSPECIMEN_PITCH_POS = 36.4;
    public static double LOWSPECIMEN_EXTEND_POS = 10;
    public static double LOWSPECIMEN_PITCH_POS = 25;
    public static double HIGHSPECIMEN_TELE_EXTEND_POS = 14.5;
//    public static double HIGHSPECIMEN_TELE_PITCH_POS = 40.5;
    public static double HIGHSPECIMEN_TELE_PITCH_POS = 36.4;
    public static double LOWSPECIMEN_TELE_EXTEND_POS = 10;
    public static double LOWSPECIMEN_TELE_PITCH_POS = 25;
    public static double SPECIMENGRAB_EXTEND_POS = 0;
    public static double SPECIMENGRAB_PITCH_POS = 165;
    public static double HOVER_EXTEND_POS = 12;
    public static double HOVER_PITCH_POS = 15;
    public static double HANG_EXTEND_POS = 5;
    public static double HANG_PITCH_POS = 70, RETRACTED_EXTEND__POS = 0, RETRACTED_PITCH_POS = 0,
            MANUAL_EXT_SPEED = 0.75, MANUAL_ROT_SPEED = 2.5, EXP_HEIGHT_OFFSET=3,
            AUTO_GRAB_PITCH = 7, AUTO_GRAB_EXTEND =14, AUTO_AUTO_GRAB_PITCH = 8,
            BACK_PITCH_POS = 97, BACK_EXTEND_POS = 15, FRONT_PITCH_POS = 32, FRONT_EXTEND_POS = 0;

    private final double EXTEND_MOTOR_BUFFER = 5;
    private final double PITCH_MOTOR_BUFFER = 3;

    double lastManualTime = -100;

    public static volatile double expectedHeight = 2, angle;

    boolean manualLoop = false;

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
            ArmStates.HIGH_SPECIMEN.pitchPos = HIGHSPECIMEN_PITCH_POS;
            ArmStates.HIGH_SPECIMEN.extendPos = HIGHSPECIMEN_EXTEND_POS;
            ArmTargetStates.HIGH_SPECIMEN.pitchPos = HIGHSPECIMEN_PITCH_POS;
            ArmTargetStates.HIGH_SPECIMEN.extendPos = HIGHSPECIMEN_EXTEND_POS;
            ArmStates.SPECIMEN_GRAB.pitchPos = SPECIMENGRAB_PITCH_POS;
        } else{
            ArmStates.HIGH_SPECIMEN.pitchPos = HIGHSPECIMEN_TELE_PITCH_POS;
            ArmStates.HIGH_SPECIMEN.extendPos = HIGHSPECIMEN_TELE_EXTEND_POS;
            ArmTargetStates.HIGH_SPECIMEN.pitchPos = LOWSPECIMEN_TELE_PITCH_POS;
            ArmTargetStates.HIGH_SPECIMEN.extendPos = LOWSPECIMEN_TELE_EXTEND_POS;
            ArmStates.HIGH_BUCKET.extendPos = HIGHBUCKET_EXTEND_POS;
            ArmTargetStates.HIGH_BUCKET.extendPos = HIGHBUCKET_EXTEND_POS;
            ArmStates.SPECIMEN_GRAB.pitchPos = SPECIMENGRAB_PITCH_POS;
        }
        ArmStates.AUTO_GRAB.extendPos=AUTO_GRAB_EXTEND;
        ArmStates.AUTO_GRAB.pitchPos = AUTO_GRAB_PITCH;
        ArmTargetStates.AUTO_GRAB.extendPos=AUTO_GRAB_EXTEND;
        ArmTargetStates.AUTO_GRAB.pitchPos = AUTO_GRAB_PITCH;
        if(!isTeleop)
        {
            ArmStates.AUTO_GRAB.pitchPos = AUTO_AUTO_GRAB_PITCH;
            ArmTargetStates.AUTO_GRAB.pitchPos = AUTO_AUTO_GRAB_PITCH;

        }
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

        HANG(false, HANG_EXTEND_POS, HANG_PITCH_POS),
        AUTO_GRAB(false, AUTO_GRAB_EXTEND, AUTO_GRAB_PITCH),
        BACK_SPECIMEN(false, BACK_EXTEND_POS, BACK_PITCH_POS),
        FRONT_WALL(false, FRONT_EXTEND_POS, FRONT_PITCH_POS);

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
        public double getExtendPos(){
            return this.extendPos;
        }
        public double getPitchPos(){
            return this.pitchPos;
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
        HANG(false, HANG_EXTEND_POS, HANG_PITCH_POS),
        AUTO_GRAB(false, AUTO_GRAB_EXTEND, AUTO_GRAB_PITCH),
        BACK_SPECIMEN(false, BACK_EXTEND_POS, BACK_PITCH_POS),
        FRONT_WALL(false, FRONT_EXTEND_POS, FRONT_PITCH_POS);;


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
        if(!ArmStates.HOVER.getState() || p_pitch>0.1) {
            super.goTo(super.getTargetExt()+p_extend*MANUAL_EXT_SPEED, super.getTargetRot()+p_pitch*MANUAL_ROT_SPEED);
            for (var i : TelescopicArm.ArmTargetStates.values()) {
                i.state = false;
            }
        } else{
            super.goTo(super.getTargetExt()+p_extend*MANUAL_EXT_SPEED, Math.atan2(3, super.getTargetExt()+p_extend*MANUAL_EXT_SPEED+10)*180/PI);
        }
    }
    public void lowerToIntake(){
        super.goTo((super.getTargetExt()+4)*cos((super.getTargetRot())*PI/180)-4,0);
        ArmStates.INTAKE.setStateTrue();
    }

    public void goTo(TelescopicArm.ArmStates p_state) {
        if(p_state == ArmStates.RETRACTED){
            if((Claw.ClawStates.OPEN.getState()||Claw.ClawStates.GIGA_OPEN.getState())&&!Claw.ClawTargetStates.CLOSED.getState()&&!ArmStates.HIGH_SPECIMEN.state){
                super.goTo(p_state.extendPos, 0,0, getRot());
            }
            else{
                super.goTo(p_state.extendPos, 100, 0, getRot());
            }
            return;
        }
        if(ArmStates.RETRACTED.getState()){
            double middle = 0;
            if(abs(p_state.pitchPos-getRot())>30){
                middle = 0;
            }
            else{
                middle = min(p_state.extendPos, getExt());
            }
            super.goTo(p_state.extendPos,p_state.pitchPos,middle, p_state.pitchPos);
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
            super.goTo(p_state.extendPos, p_state.pitchPos, middle, ArmStates.HIGH_SPECIMEN.pitchPos);
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
    public void goTo(double p_extend, double p_pitch, double middle, double middleRot) {
        super.goTo(p_extend, p_pitch, middle, middleRot);
    }
    public double getVel(){
        return super.getVel();
    }

    public void goToResetManual(double p_extend, double p_pitch){
        lastManualTime = -100;
        goTo(p_extend,p_pitch);
    }
    public void setPowers(double ext, double rot){
        super.setPowers(ext, rot);
    }
    public void setManualPowers(double ext, double rot){
//        manualLoop = true;
        super.setPowers(ext, rot);
    }

    /**
     * updates the state machine
     * TODO: ZEROING SYSTEM
     */
    public void update() {
//        if(!manualLoop) {
            boolean targeted = false;
            if (time - lastManualTime > .5) {
                for (var i : TelescopicArm.ArmStates.values()) {
                    if (abs(super.getExt() - i.extendPos) < EXTEND_MOTOR_BUFFER && abs(super.getRot() - i.pitchPos) < PITCH_MOTOR_BUFFER) {
                        i.state = true;
                        TelescopicArm.ArmTargetStates.values()[i.ordinal()].state = false;
                    } else {
                        i.state = false;
                    }
                }
                if (abs(super.getExt()) < 3) {
                    ArmStates.RETRACTED.state = true;
                    ArmTargetStates.RETRACTED.state = false;
                }
                if (abs(getRot() - HIGHSPECIMEN_PITCH_POS) < 10 && abs(getExt() - HIGHSPECIMEN_EXTEND_POS) < 3) {
                    ArmStates.HIGH_SPECIMEN.state = true;
                    TelescopicArm.ArmTargetStates.values()[ArmStates.HIGH_SPECIMEN.ordinal()].state = false;

                }
                if (getRot() < 5 && getTargetRot() < 1) {
                    ArmStates.INTAKE.state = true;

                }
                if (abs(3 - (getExt() + 10) * sin((getRot()) * PI / 180)) < 3.75 && getRot() < 90) {
                    ArmStates.HOVER.state = true;
                    TelescopicArm.ArmTargetStates.values()[ArmStates.HOVER.ordinal()].state = false;
                } else {
                    ArmStates.HOVER.state = false;
                }
                for (var i : TelescopicArm.ArmTargetStates.values()) {
                    if (i.state && abs(super.getExt() - i.extendPos) > EXTEND_MOTOR_BUFFER && abs(super.getRot() - i.pitchPos) > PITCH_MOTOR_BUFFER && (isMid() && abs(getExt() - getTargetExt()) < 3) && abs(getRot() - getTargetRot()) < 10) {
//                    goTo(TelescopicArm.ArmStates.values()[i.ordinal()]);
                        targeted = true;
                    }
                }

            }
            if (!targeted && isMid())
                super.goTo(getTargetExt(), getTargetRot());
            else if (!isMid())
                super.goTo(getTrueTargExt(), getTrueTargRot(), getMiddle(), getMiddleRot());
            expectedHeight = sin(getRot() * PI / 180) * (10 + getExt()) + EXP_HEIGHT_OFFSET;
            angle = getRot();
            packet.put("targExt", super.getTargetExt());
            packet.put("expH", sin(getRot() * PI / 180) * (10 + getExt()) + EXP_HEIGHT_OFFSET);
            packet.put("targRot", super.getTargetRot());
//        packet.put("targMid", super.getMiddle());
//        packet.put("targMidRot", super.getMiddleRot());
            packet.put("curExt", super.getExt());
            packet.put("curRot", super.getRot());
//        packet.put("horiExt", super.getExt()*cos(getRot()*PI/180));
//        packet.put("targVertExt",(getTargetExt()+12)*sin(getTargetRot()*PI/180));
            packet.put("curVertExt", (getExt() + 10) * sin((getRot()) * PI / 180));
//        packet.put("diff",abs(5-(getTargetExt()+10)*sin(getRot()*PI/180)));
//        packet.put("boolean", abs(5-(getTargetExt()+10)*sin(getRot()*PI/180))>4);
//        packet.put("targeted", targeted);
//        packet.put("isMid", isMid());
//        packet.put("HOVER", ArmStates.HOVER.getState());
//        packet.put("hoverbuffa",abs(super.getRot()-HOVER_PITCH_POS));
//        }
//        else{
//            manualLoop = false;
//        }

    }

}
