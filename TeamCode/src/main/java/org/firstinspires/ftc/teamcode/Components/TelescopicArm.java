package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static java.lang.Math.abs;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.DualPIDController;

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
    public static double HANG_PITCH_POS = 0, RETRACTED_EXTEND__POS = 0, RETRACTED_PITCH_POS;

    private final double EXTEND_MOTOR_BUFFER = 0;
    private final double PITCH_MOTOR_BUFFER = 0;

    double lastManualTime = -100;

    /**
     * init
     */
    public TelescopicArm() {
        super();
        lastManualTime = -100;
    }

    /**
     * possible states of arm
     */
    public enum ArmStates {
        INTAKE(false, INTAKE_EXTEND_POS, INTAKE_PITCH_POS),
        HIGH_BUCKET(false, HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS),
        LOW_BUCKET(false, LOWBUCKET_EXTEND_POS, LOWBUCKET_PITCH_POS),
        HIGH_SPECIMEN(false, HIGHSPECIMEN_EXTEND_POS, HIGHSPECIMEN_PITCH_POS),
        LOW_SPECIMEN(false, LOWSPECIMEN_EXTEND_POS, LOWSPECIMEN_PITCH_POS),
        SPECIMEN_GRAB(false, SPECIMENGRAB_EXTEND_POS, SPECIMENGRAB_PITCH_POS),
        SAMPLE_DROP(false, SAMPLEDROP_EXTEND_POS, SAMPLEDROP_PITCH_POS),
        HOVER(false, HOVER_EXTEND_POS, HOVER_PITCH_POS),
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
        LOW_SPECIMEN(false, LOWSPECIMEN_EXTEND_POS, LOWSPECIMEN_PITCH_POS),
        HIGH_SPECIMEN(false, HIGHSPECIMEN_EXTEND_POS, HIGHSPECIMEN_PITCH_POS),
        HIGH_BUCKET(false, HIGHBUCKET_EXTEND_POS, HIGHBUCKET_PITCH_POS),
        LOW_BUCKET(false, LOWBUCKET_EXTEND_POS, LOWBUCKET_PITCH_POS),
        SPECIMEN_GRAB(false, SPECIMENGRAB_EXTEND_POS, SPECIMENGRAB_PITCH_POS),

        RETRACTED(true, RETRACTED_EXTEND__POS, RETRACTED_PITCH_POS),
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
        super.goTo(p_extend, p_pitch);
        for (var i : TelescopicArm.ArmTargetStates.values()) {
            i.state = false;
        }
    }

    public void goTo(TelescopicArm.ArmStates p_state) {
        if(p_state == ArmStates.RETRACTED){
            if(Claw.ClawStates.OPEN.getState()){
                super.goTo(p_state.extendPos, 0);
            }
            else{
                super.goTo(p_state.extendPos, 90);
            }
        }
        if (ArmStates.INTAKE.getState() || ArmStates.LOW_SPECIMEN.getState() || ArmStates.HOVER.getState()) {
            if (p_state == ArmStates.HOVER || p_state == ArmStates.INTAKE || p_state == ArmStates.LOW_SPECIMEN) {
                super.goTo(p_state.extendPos, p_state.pitchPos);
            } else {
                super.goTo(p_state.extendPos, p_state.pitchPos, 0, ArmStates.HOVER.pitchPos);
            }
        } else if (ArmStates.HIGH_SPECIMEN.getState()) {
            super.goTo(p_state.extendPos, p_state.pitchPos, 0, ArmStates.HIGH_SPECIMEN.pitchPos - 5);
        } else if (ArmStates.HIGH_BUCKET.getState()) {
            if (p_state == ArmStates.LOW_BUCKET || p_state == ArmStates.SPECIMEN_GRAB) {
                super.goTo(p_state.extendPos, p_state.pitchPos, p_state.extendPos, ArmStates.HIGH_BUCKET.pitchPos);
            } else {
                super.goTo(p_state.extendPos, p_state.pitchPos, 0, p_state.pitchPos);
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

    /**
     * updates the state machine
     */
    public void update() {
        boolean targeted = false;
        if (time - lastManualTime > .5) {
            for (var i : TelescopicArm.ArmStates.values()) {
                if (abs(super.getExtPosition() - i.extendPos) < EXTEND_MOTOR_BUFFER && abs(super.getRotPosition() - i.pitchPos) < PITCH_MOTOR_BUFFER) {
                    i.setStateTrue();
                    TelescopicArm.ArmTargetStates.values()[i.ordinal()].state = false;
                }
                if(i == ArmStates.RETRACTED){
                    if(abs(super.getExtPosition())<3){
                        i.setStateTrue();
                        TelescopicArm.ArmTargetStates.values()[i.ordinal()].state = false;
                    }
                }
            }
            for (var i : TelescopicArm.ArmTargetStates.values()) {
                if (i.state && abs(super.getExtPosition() - i.extendPos) > EXTEND_MOTOR_BUFFER && abs(super.getRotPosition() - i.pitchPos) > PITCH_MOTOR_BUFFER) {
                    goTo(TelescopicArm.ArmStates.values()[i.ordinal()]);
                    targeted = true;
                }
            }

        }
        if (!targeted)
            goTo(getTargetExt(), getTargetRot());
    }
}
