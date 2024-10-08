package org.firstinspires.ftc.teamcode.Components;

import static org.apache.commons.math3.util.FastMath.abs;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isFlipped;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;

import static java.lang.Math.max;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFDualMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

/** William */
@Config
public class Lift extends RFDualMotor {
  private double lastPower = 0.0;
  private double target = 0.0;
  private double MIN_VELOCITY = 20, MANUAL_TIME = 0.2, lastManualTime = -1.0;
  private int iterateHeight = 6;
  public static double max = 1800,
      min = -15,
      RESISTANCE = 640,
      kS = 0.06,
      kV = 3.2786E-4,
      kA = 6E-5,
      MAX_UP_VELO = 3000,
      MAX_DOWN_VELO = -2080,
      MAX_ACCEL = 6000,
      MAX_DECEL = -6000,
      kP = 0,
      kD = 0, liftHeight = 0;
  double[] liftPositions = {550,680,800,1000,1100,1200,1300,1400,1800};

  /** Constructor */
  public Lift() {
    super("leftLiftMotor", "rightLiftMotor", !isTeleop);
    super.setDirection(DcMotorSimple.Direction.REVERSE);
    super.setDirection2(DcMotorSimple.Direction.FORWARD);
    setConstants(
        max, min, RESISTANCE, kS, kV, kA, MAX_UP_VELO, MAX_DOWN_VELO, MAX_ACCEL, MAX_DECEL, kP, kD);
    if (isFlipped) {
      super.setTarget(800);
      LiftMovingStates.LOW.setStateTrue();
      LiftPositionStates.LOW_SET_LINE.setStateTrue();
    }
    else{
      super.setTarget(0);
      LiftMovingStates.AT_ZERO.setStateTrue();
    }
    lastPower = 0;
    lastManualTime = -100;
    target = 0;
  }

  /** Stores different states of lift. */
  public enum LiftPositionStates {
    HIGH_SET_LINE(1600, false),
    MID_SET_LINE(1050, false),
    LOW_SET_LINE(800, false),
    AT_ZERO(0, true);

    double position;
    boolean state;

    LiftPositionStates(double p_position, boolean p_state) {
      this.position = p_position;
      this.state = p_state;
    }

    void setStateTrue() {
      if (!this.state) {
        for (int i = 0; i < LiftPositionStates.values().length; i++) {
          if (LiftPositionStates.values()[i].state) {
            LOGGER.log(
                "assigned false to position state: " + LiftPositionStates.values()[i].name());
          }
          LiftPositionStates.values()[i].state = false;
        }
        this.state = true;
        LOGGER.log(RFLogger.Severity.INFO, "assigned true to position state: " + this.name());
      }
    }

    public boolean getState() {
      return this.state;
    }

    public double getPosition() {
      return position;
    }
  }

  public enum LiftMovingStates {
    HIGH(false),
    MID(false),
    LOW(false),
    AT_ZERO(true);

    public boolean state;

    LiftMovingStates(boolean p_state) {
      this.state = p_state;
    }

    void setStateTrue() {
      if (!this.state) {
        for (int i = 0; i < LiftMovingStates.values().length; i++) {
          if (LiftMovingStates.values()[i].state) {
            LOGGER.log("assigned false to target state: " + LiftMovingStates.values()[i].name());
          }
          LiftMovingStates.values()[i].state = false;
        }
        this.state = true;
        LOGGER.log(RFLogger.Severity.INFO, "assigned true to target state: " + this.name());
      }
    }

    public boolean getState() {
      return this.state;
    }

    public void clearTargets() {
      for (LiftMovingStates i : LiftMovingStates.values()) {
        i.state = false;
      }
    }
  }

  /**
   * Depending on which state the lift is currently in, checks whether the state can be transitioned
   * to the next state, then changes state values. Logs which state(s)' values have been changed and
   * to what. Logs to RFMotor & general logs. Logs to finest level. Updates LiftMovingStates and
   * LiftPositionStates state machines.
   */
  public void update() {
//    LOGGER.log(
//        RFLogger.Severity.FINEST,
//        "currentPos: " + super.getCurrentPosition() + ", currentTarget: " + super.getTarget());
    liftHeight = super.getCurrentPosition();
    packet.put("liftPos", super.getCurrentPosition());
    for (var i : LiftPositionStates.values()) {
      if (abs(super.getCurrentPosition() - i.position) < 50
          && (i != LiftPositionStates.AT_ZERO
              || (Arm.ArmStates.HOVER.getState() || Arm.ArmStates.GRAB.getState()))) {
        i.setStateTrue();
        LiftMovingStates.values()[i.ordinal()].state=false;
      }
    }

    if (super.getCurrentPosition() < LiftPositionStates.LOW_SET_LINE.position-70-getVelocity()*0.4)
      LiftPositionStates.AT_ZERO.setStateTrue();
    else LiftPositionStates.AT_ZERO.state = false;

    for (var i : LiftMovingStates.values()) {
      if (i.state
          && abs(super.getTarget() - LiftPositionStates.values()[i.ordinal()].position) > 30) {
        LOGGER.log("update to state:"+ i.name());
        setPosition(LiftPositionStates.values()[i.ordinal()]);
        break;
      }
    }
      if (time - lastManualTime > MANUAL_TIME) {
        if(super.getCurrentPosition()<30 && super.getTarget()<10){
          super.setRawPower(-0.3);
        } else {
          setPosition(super.getTarget(), 0);
        }
      } else {
        setTarget(super.getCurrentPosition());
        LiftMovingStates.LOW.clearTargets();
      }
//    LOGGER.log(RFLogger.Severity.FINE, "currentPos: " + super.getCurrentPosition());
//    packet.put("liftPos", super.getCurrentPosition());
  }

  /**
   * Sets target position for lift.
   *
   * @param p_target target position for lift to run to Logs what position the target position has
   *     been set to. Logs to RFMotor & general logs. Logs to finest level. Updates LiftMovingStates
   *     state machine.
   */
  public void setPosition(double p_target) {
    super.setPosition(p_target, 0);
    if (target != p_target) {
      LOGGER.setLogLevel(RFLogger.Severity.INFO);
      LOGGER.log("lifting to: " + p_target);
      target = p_target;
    }
  }

  public void setPosition(LiftPositionStates p_state) {
    LOGGER.log("lifting to: " + p_state.name());
    if (p_state.equals(LiftPositionStates.AT_ZERO)) {
      if ((Arm.ArmStates.HOVER.getState() || Arm.ArmStates.GRAB.state)
          && (!Arm.ArmTargetStates.DROP.state || !Arm.ArmTargetStates.GRAB.getState())) {
        super.setPosition(p_state.position, 0);
        LiftMovingStates.LOW.clearTargets();
        LiftMovingStates.AT_ZERO.setStateTrue();
        LOGGER.log("HOVER: "+Arm.ArmStates.HOVER.state);
        LOGGER.log("Targ_DROp: "+Arm.ArmTargetStates.DROP.state);
        LOGGER.log("TargGRAB : "+Arm.ArmTargetStates.GRAB.state);



      } else {
        super.setPosition(LiftPositionStates.LOW_SET_LINE.position, 0);
        LOGGER.log("Out");
      }
      iterateHeight=6;
    } else {
      if (!Arm.ArmStates.GRAB.getState())
        super.setPosition(p_state.position, 0);
    }

      LiftMovingStates.values()[p_state.ordinal()].state = true;
  }

  /**
   * Manually extend/retract slides
   *
   * @param p_power How fast the user wants to move the slides and in what direction Logs that the
   *     lift is currently being manually extended. Logs to RFMotor & general logs. Logs to finest
   *     level. Updates LiftMovingStates state machine.
   */
  public void manualExtend(double p_power) {
    super.setPower(p_power*=.5);
    lastManualTime = time;
    if (p_power != lastPower) {
      LOGGER.setLogLevel(RFLogger.Severity.INFO);
      LOGGER.log("setting power to: " + p_power);
      lastPower = p_power;
    }
  }

  /**
   * Iterate to the next highest set line lift height. Logs what set line lift target height the
   * lift has been set to run to. Logs to RFMotor & general logs. Logs to finest level. Updates
   * LiftMovingStates state machine.
   */
  public void iterateUp() {
    iterateHeight--;
    iterateHeight= max(iterateHeight,0);

    setPosition(liftPositions[8-iterateHeight]);
//    LOGGER.log("iterated up to state: " + LiftPositionStates.values()[iterateHeight]);
  }

  /**
   * Iterate to the next lowest set line lift height. Logs what set line lift target height the lift
   * has been set to run to. Logs to RFMotor & general logs. Logs to finest level. Updates
   * LiftMovingStates state machine.
   */
  public void iterateDown() {
    iterateHeight++;
    iterateHeight = min(iterateHeight, 8);
    setPosition(liftPositions[8-iterateHeight]);
//    LOGGER.log("iterated down to state: " + LiftPositionStates.values()[iterateHeight]);
    }
}
