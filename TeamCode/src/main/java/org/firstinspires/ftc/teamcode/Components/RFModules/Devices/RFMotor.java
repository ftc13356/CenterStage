package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.time;
import static java.lang.Double.max;
import static java.lang.Double.min;
import static java.lang.Math.abs;
import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorImpl;
import com.qualcomm.robotcore.hardware.DcMotorImplEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Robots.BasicRobot;
import org.firstinspires.ftc.teamcode.roadrunner.util.DashboardUtil;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Harry
 */
@Config
public class RFMotor {
    private final DcMotorEx rfMotor;
    double kP = 0.009;
    double kD = 0.00001;
    double kV = 0.0003;
    double kA = 0.00004;
    double kS = 0.15;
    double MAX_ACCELERATION_UP = 6000;
    double MAX_ACCELERATION_DOWN = 12000;
    double RESISTANCE = 0;
    double gravity = 0.2;
    double MAX_VELOCITY_UP = 1475 - 225 * (13.5 - BasicRobot.voltageSensor.getVoltage());
    private double MAX_VELOCITY_DOWN = 3500;
    private double relativeDist, direction, peakVelo, J, decelDist;
    private final double[][] calculatedIntervals = new double[4][8];
    private final double[][][] calculatedMotions = new double[3][7][5];
    private double[] timeIntervals = new double[8];
    private double[] distances = new double[8];
    private double[] velocities = new double[8];
    private double[] positions = new double[8];
    private double maxtickcount = 10000;
    private double mintickcount = 0;
    private final double DEFAULTCOEF1 = 0.0001;
    private final double DEFAULTCOEF2 = 0.01;
    private final double lastError = 0;
    private double lastTime = 0;
    private double additionalTicks = 0;
    private double TICK_BOUNDARY_PADDING = 20, TICK_STOP_PADDING = 20;

    private double currentAcceleration, currentPos, currentTickPos;
    private double power = 0, position = 0, velocity = 0, targetPos = 0, resistance = 0, acceleration = 0;
    private String rfMotorName;
    private boolean isSim = false, sameTarget = false;
    private double startPosition =0;
    RFMotionProfile profile;

    /**
     * Constructor
     *
     * @param p_motorName      the name of the device
     * @param p_motorDirection the direction of the motor | 0 for Reverse, 1 for Forward
     * @param p_resetPos       if true, motor encoder is reset to 0; if false, nothing happens
     * @param p_maxtick        max allowed tick position of the motor
     * @param p_mintick        minimum allowed tick position of the motor
     */
    public RFMotor(String p_motorName, DcMotorSimple.Direction p_motorDirection, boolean p_resetPos, double p_maxtick,
                   double p_mintick) {
        rfMotor = (DcMotorEx) op.hardwareMap.dcMotor.get(p_motorName);
        rfMotor.setDirection(p_motorDirection);
        if (p_resetPos) {
            rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
        maxtickcount = p_maxtick;
        mintickcount = p_mintick;

        logger.createFile("/MotorLogs/RFMotor" + p_motorName, "Runtime    Component               " +
                "Function               Action");
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        additionalTicks = 0;
        resistance= RESISTANCE;
    }

    /**
     * Constructor
     *
     * @param p_motorName the name of the device
     * @param p_resetPos  if true, motor encoder is reset to 0; if false, nothing happens
     * @param p_maxtick   max allowed tick position of the motor
     * @param p_mintick   minimum allowed tick position of the motor
     */
    public RFMotor(String p_motorName, boolean p_resetPos, double p_maxtick, double p_mintick) {
        this(p_motorName, DcMotorSimple.Direction.FORWARD, p_resetPos, p_maxtick, p_mintick);
    }

    /**
     * Constructor
     *
     * @param p_motorName the name of the device
     * @param p_resetPos  if true, motor encoder is reset to 0; if false, nothing happens
     */
    public RFMotor(String p_motorName, boolean p_resetPos) {
        this(p_motorName, DcMotorSimple.Direction.FORWARD,p_resetPos, 1000,0);
    }

    /**
     * Setting target position of the motor
     *
     * @param p_targetPos target position for motor to travel to
     * @param p_curve     desired curviness of S-Curve
     */
    public void setPosition(double p_targetPos, double p_curve) {
        if (abs(targetPos - p_targetPos) < 25) {
            sameTarget = true;
        } else {
            sameTarget = false;
        }

        power = 0;
        if (p_targetPos > maxtickcount) {
            p_targetPos = maxtickcount;
        }
        if (p_targetPos < mintickcount) {
            p_targetPos = mintickcount;
        }
        position = rfMotor.getCurrentPosition();
        targetPos = p_targetPos;
        acceleration = rfMotor.getVelocity() - velocity;
        velocity += acceleration;
        acceleration /= (time - lastTime);
        if (!sameTarget) {
            direction = Math.signum(targetPos - position);
//            LOGGER.log("target" + p_targetPos + ", lastTarg" + targetPos + " profile is null:" + String.valueOf(profile == null) + " sameTarget:" + sameTarget);
            profile = new RFMotionProfile(direction*velocity, 0, 0, MAX_ACCELERATION_UP, MAX_VELOCITY_UP);
            profile.calculateTList(abs(targetPos - position));
            startPosition=position;

        }
        if(profile==null){
            position=-3;
            direction = Math.signum(targetPos - position);
//            LOGGER.log("target" + p_targetPos + ", lastTarg" + targetPos + " profile is null:" + String.valueOf(profile == null) + " sameTarget:" + sameTarget);
            profile = new RFMotionProfile(direction*velocity, 0, 0, MAX_ACCELERATION_UP, MAX_VELOCITY_UP);
            profile.calculateTList(abs(targetPos - position));
            startPosition=position;        }
        double[] targetMotion = new double[]{velocity, direction * profile.getInstantaneousTargetAcceleration(direction
                        * (targetPos - position), direction * velocity, 0)};
//        LOGGER.log("kA:"+ kA+" kV:"+kV);
//        LOGGER.log("targetMotion[0]:"+ targetMotion[0]+" targetMotion[1]:"+targetMotion[1]);
        double power = (kV * targetMotion[0] + kA * targetMotion[1] +
                kP * (profile.motionProfileTimeToDist(time) - position)
                + kD * (profile.calculateTargetVelocity(time) - velocity)
                + resistance * kV);
        if (abs(targetPos - position) > TICK_BOUNDARY_PADDING && abs(velocity) < 3) {
            if (power < 0) {
                power -= kS;
            } else {
                power += kS;
            }
        }
        if(abs(targetPos - position)<TICK_BOUNDARY_PADDING){
            power = resistance*kV;
        }
//        LOGGER.log("motor power :" + power+" pos " + position + " targetPos " + targetPos);
        setRawPower(power);
        lastTime = time;
    }

    /**
     * Gets target current velocity and target current acceleration
     *
     * @param p_curve desired curviness of S-Curve
     * @return target current velocity and target current acceleration
     */
    public double[] getTargetMotion(double p_curve) {
        double[] targets = {0, 0};

        relativeDist = targetPos - getCurrentPosition();

        if (relativeDist == 0) {
            return targets;
        }

        if (sameTarget) {
            targets[0] = getTargetVelocity(BasicRobot.time);
            targets[1] = getTargetAcceleration(BasicRobot.time);
            return targets;
        }

        direction = abs(relativeDist) / relativeDist;

        if (isSim) {
            velocities[0] = direction * velocity;
            positions[0] = direction * currentTickPos;
        } else {
            velocities[0] = direction * rfMotor.getVelocity();
            positions[0] = direction * rfMotor.getCurrentPosition();
        }

        if (velocities[0] == 0) {
            peakVelo = min((131 - 38 * p_curve) / 131 * sqrt(getMaxAcceleration() * abs(relativeDist)), getMaxVelocity());
        } else {
            peakVelo = min((131 - 38 * p_curve) / 131 * sqrt(getMaxAcceleration() * (abs(relativeDist) -
                    abs(velocities[0]) / velocities[0] * pow(velocities[0], 2) / (2 * getMaxAcceleration()))), getMaxVelocity());
        }

        J = getMaxAcceleration() / ((peakVelo / (getMaxAcceleration() * (1 - p_curve / 2))) * p_curve / 2);

        velocities[0] = min(velocities[0], peakVelo);

        calculateIntervals();
        calculateMotions();
        timeIntervals = calculatedIntervals[0];
        distances = calculatedIntervals[1];
        velocities = calculatedIntervals[2];
        positions = calculatedIntervals[3];
        decelDist = calculatedIntervals[3][7] - calculatedIntervals[3][4];

        if (isSim) {
            velocities[0] = velocity;
            positions[0] = currentTickPos;
            isSim = false;
        } else {
            velocities[0] = rfMotor.getVelocity();
            positions[0] = rfMotor.getCurrentPosition();
        }

        targets[0] = getTargetVelocity(BasicRobot.time);
        targets[1] = getTargetAcceleration(BasicRobot.time);

        decelDist = getTargetPosition(timeIntervals[7]) - getTargetPosition(timeIntervals[4]);

        return targets;
    }

    /**
     * Calculates time intervals, end velocities and positions at the end of each time interval, and distance
     * covered in each time segment, then updates the public static arrays
     */
    public void calculateIntervals() {
        double[] temp_timeIntervals = new double[8];
        double cruiseTime;
        double[] temp_distances = new double[8];
        double semiTotal;
        double cruiseAccelTime;
        double[] temp_velocities = new double[8];
        double[] temp_positions = new double[8];

        temp_timeIntervals[0] = BasicRobot.time;
        temp_timeIntervals[2] = max(sqrt((peakVelo - velocities[0]) / J), (max(0, (peakVelo - velocities[0])
                / getMaxAcceleration())));
        temp_timeIntervals[1] = min(temp_timeIntervals[2], getMaxAcceleration() / J);
        temp_timeIntervals[3] = temp_timeIntervals[1] + temp_timeIntervals[2];

        cruiseAccelTime = temp_timeIntervals[2] - temp_timeIntervals[1];

        temp_velocities[0] = velocities[0];
        temp_velocities[1] = temp_velocities[0] + J * pow(temp_timeIntervals[1], 2) / 2;
        temp_velocities[2] = temp_velocities[1] + getMaxAcceleration() * cruiseAccelTime;
        temp_velocities[3] = peakVelo;
        temp_velocities[4] = peakVelo;
        temp_velocities[5] = temp_velocities[4] - J * pow(getMaxAcceleration() / J, 2) / 2;
        temp_velocities[6] = J * pow(getMaxAcceleration() / J, 2) / 2;
        temp_velocities[7] = 0;

        temp_distances[0] = 0;
        temp_distances[1] = J * pow(temp_timeIntervals[1], 3) / 6 + temp_velocities[0] * temp_timeIntervals[1];
        temp_distances[2] = temp_velocities[1] * cruiseAccelTime + getMaxAcceleration() * pow(cruiseAccelTime, 2) / 2;
        temp_distances[3] = temp_velocities[2] * temp_timeIntervals[1] + J * pow(temp_timeIntervals[1], 3) / 6;
        temp_distances[5] = temp_velocities[4] * getMaxAcceleration() / J - J * pow(getMaxAcceleration() / J, 3) / 6;
        temp_distances[6] = temp_velocities[5] * (peakVelo / getMaxAcceleration() - getMaxAcceleration() / J) -
                getMaxAcceleration() * pow(peakVelo / getMaxAcceleration() - getMaxAcceleration() / J, 2) / 2;
        temp_distances[7] = J * pow(getMaxAcceleration() / J, 3) / 6;

        semiTotal = temp_distances[1] + temp_distances[2] + temp_distances[3] + temp_distances[5] +
                temp_distances[6] + temp_distances[7];

        cruiseTime = (abs(relativeDist) - semiTotal) / peakVelo;

        temp_timeIntervals[4] = temp_timeIntervals[3] + cruiseTime;
        temp_timeIntervals[5] = temp_timeIntervals[4] + getMaxAcceleration() / J;
        temp_timeIntervals[6] = temp_timeIntervals[4] + peakVelo / getMaxAcceleration();
        temp_timeIntervals[7] = temp_timeIntervals[6] + getMaxAcceleration() / J;

        temp_distances[4] = temp_velocities[3] * cruiseTime;

        temp_positions[1] = temp_positions[0] + temp_distances[1];
        temp_positions[2] = temp_positions[1] + temp_distances[2];
        temp_positions[3] = temp_positions[2] + temp_distances[3];
        temp_positions[4] = temp_positions[3] + temp_distances[4];
        temp_positions[5] = temp_positions[4] + temp_distances[5];
        temp_positions[6] = temp_positions[5] + temp_distances[6];
        temp_positions[7] = temp_positions[6] + temp_distances[7];

        calculatedIntervals[0] = temp_timeIntervals;
        calculatedIntervals[1] = temp_distances;
        calculatedIntervals[2] = temp_velocities;
        calculatedIntervals[3] = temp_positions;
    }

    /**
     * Stores coefficients for calculating target position, velocity, or acceleration at any given time
     */
    public void calculateMotions() {
        double[][] acceleration = new double[7][5];
        double[][] velocity = new double[7][5];
        double[][] position = new double[7][5];

        for (double[] type : acceleration) {
            Arrays.fill(type, -1);
        }

        for (double[] type : velocity) {
            Arrays.fill(type, -1);
        }

        for (double[] type : position) {
            Arrays.fill(type, -1);
        }

        acceleration[0][1] = J;

        acceleration[1][0] = J * calculatedIntervals[0][1];

        acceleration[2][1] = -J;
        acceleration[2][4] = calculatedIntervals[0][3];

        acceleration[4][1] = -J;
        acceleration[4][4] = calculatedIntervals[0][4];

        acceleration[5][0] = -getMaxAcceleration();

        acceleration[6][1] = J;
        acceleration[6][4] = calculatedIntervals[0][7];

        velocity[0][0] = calculatedIntervals[2][0];
        velocity[0][2] = J / 2;

        velocity[1][0] = calculatedIntervals[2][1];
        velocity[1][1] = getMaxAcceleration();
        velocity[1][4] = calculatedIntervals[0][1];

        velocity[2][0] = calculatedIntervals[2][2] + J / 2 * pow(calculatedIntervals[0][1], 2);
        velocity[2][2] = -J / 2;
        velocity[2][4] = calculatedIntervals[0][3];

        velocity[3][0] = calculatedIntervals[2][3];

        velocity[4][0] = calculatedIntervals[2][4];
        velocity[4][2] = -J / 2;
        velocity[4][4] = calculatedIntervals[0][4];

        velocity[5][0] = calculatedIntervals[2][5];
        velocity[5][1] = -getMaxAcceleration();
        velocity[5][4] = calculatedIntervals[0][5];

        velocity[6][2] = J / 2;
        velocity[6][4] = calculatedIntervals[0][7];


        position[0][0] = calculatedIntervals[3][0];
        position[0][1] = calculatedIntervals[2][0];
        position[0][3] = J / 6;

        position[1][0] = calculatedIntervals[3][1];
        position[1][1] = calculatedIntervals[2][1];
        position[1][2] = getMaxAcceleration() / 2;
        position[1][4] = calculatedIntervals[0][1];

        position[2][0] = calculatedIntervals[3][2];
        position[2][1] = calculatedIntervals[2][2];
        position[2][3] = J / 6;
        position[2][4] = calculatedIntervals[0][2];

        position[3][0] = calculatedIntervals[3][3];
        position[3][1] = calculatedIntervals[2][3];
        position[3][4] = calculatedIntervals[0][3];

        position[4][0] = calculatedIntervals[3][4];
        position[4][1] = calculatedIntervals[2][4];
        position[4][3] = -J / 6;
        position[4][4] = calculatedIntervals[0][4];

        position[5][0] = calculatedIntervals[3][5];
        position[5][1] = calculatedIntervals[2][5];
        position[5][2] = -getMaxAcceleration() / 2;
        position[5][4] = calculatedIntervals[0][5];

        position[6][0] = calculatedIntervals[3][7];
        position[6][3] = J / 6;
        position[6][4] = calculatedIntervals[0][7];

        calculatedMotions[0] = acceleration;
        calculatedMotions[1] = velocity;
        calculatedMotions[2] = position;

    }

    public boolean isDone() {
        return abs(position - targetPos) < 20;
    }

    public void setConstants(double p_max, double p_min, double p_resistance, double p_kS, double p_kV, double p_kA, double p_maxUpVelo,
                             double p_maxDownVelo, double p_maxAccel, double p_maxDecel, double p_kP, double p_kD) {
        maxtickcount = p_max;
        mintickcount = p_min;
        RESISTANCE = p_resistance;
        resistance=RESISTANCE;
        kS = p_kS;
        kV = p_kV;
        kA = p_kA;
        MAX_VELOCITY_UP = p_maxUpVelo;
        MAX_VELOCITY_DOWN = p_maxDownVelo;
        MAX_ACCELERATION_UP = p_maxAccel;
        MAX_ACCELERATION_DOWN = p_maxDecel;
        kP = p_kP;
        kD = p_kD;
    }

    /**
     * Calculates current target position given time
     *
     * @param p_time current time
     * @return
     */
    public double getTargetPosition(double p_time) {
        p_time -= timeIntervals[0];
        if (p_time >= timeIntervals[7]) {
            return positions[7] * direction + positions[0];
        }
        double currentTargetPos = 0;
        double firstInterval;
        for (int i = 0; i < 7; i++) {
            if (i == 0) {
                firstInterval = 0;
            } else {
                firstInterval = timeIntervals[i];
            }
            if (p_time >= firstInterval && p_time < timeIntervals[i + 1]) {
                for (int j = 0; j < 4; j++) {
                    if (calculatedMotions[2][i][4] != -1) {
                        if (calculatedMotions[2][i][j] != -1) {
                            currentTargetPos += pow(p_time - calculatedMotions[2][i][4], j) * calculatedMotions[2][i][j];
                        }
                    } else {
                        if (calculatedMotions[2][i][j] != -1) {
                            currentTargetPos += pow(p_time, j) * calculatedMotions[2][i][j];
                        }
                    }
                }
                break;
            }
        }

        return direction * currentTargetPos + positions[0];
    }

    /**
     * Calculates current target velcotiy given time
     *
     * @param p_time current time
     * @return
     */
    public double getTargetVelocity(double p_time) {
        p_time -= timeIntervals[0];
        if (p_time >= timeIntervals[7]) {
            return velocities[7] * direction;
        }
        double currentTargetVelo = 0;
        double firstInterval;
        for (int i = 0; i < 7; i++) {
            if (i == 0) {
                firstInterval = 0;
            } else {
                firstInterval = timeIntervals[i];
            }
            if (p_time >= firstInterval && p_time < timeIntervals[i + 1]) {
                for (int j = 0; j < 4; j++) {
                    if (calculatedMotions[1][i][4] != -1) {
                        if (calculatedMotions[1][i][j] != -1) {
                            currentTargetVelo += pow(p_time - calculatedMotions[1][i][4], j) * calculatedMotions[1][i][j];
                        }
                    } else {
                        if (calculatedMotions[1][i][j] != -1) {
                            currentTargetVelo += pow(p_time, j) * calculatedMotions[1][i][j];
                        }
                    }
                }
                break;
            }
        }
        return direction * currentTargetVelo;
    }

    /**
     * Calculates current target acceleration given time
     *
     * @param p_time current time
     * @return
     */
    public double getTargetAcceleration(double p_time) {
        p_time -= timeIntervals[0];
        if (p_time > timeIntervals[7]) {
            return 0;
        }
        double currentTargetAccel = 0;
        double firstInterval;
        for (int i = 0; i < 7; i++) {
            if (i == 0) {
                firstInterval = 0;
            } else {
                firstInterval = timeIntervals[i];
            }
            if (p_time > firstInterval && p_time < timeIntervals[i + 1]) {
                for (int j = 0; j < 4; j++) {
                    if (calculatedMotions[0][i][4] != -1) {
                        if (calculatedMotions[0][i][j] != -1) {
                            currentTargetAccel += pow(p_time - calculatedMotions[0][i][4], j) * calculatedMotions[0][i][j];
                        }
                    } else {
                        if (calculatedMotions[0][i][j] != -1) {
                            currentTargetAccel += pow(p_time, j) * calculatedMotions[0][i][j];
                        }
                    }
                }
                break;
            }
        }
        return direction * currentTargetAccel;
    }

    /**
     * Determines which MAX_VELOCITY to use based on direction of movement
     *
     * @return determined MAX_VELOCITY
     */
    public double getMaxVelocity() {
        if (direction == 1) {
            return MAX_VELOCITY_UP;
        } else {
            return MAX_VELOCITY_DOWN;
        }
    }

    /**
     * Determines which MAX_ACCEL to use based on direction of movement
     *
     * @return determined MAX_ACCEL
     */
    public double getMaxAcceleration() {
        if (direction == 1) {
            return MAX_ACCELERATION_UP;
        } else {
            return MAX_ACCELERATION_DOWN;
        }
    }

    /**
     * Returns current target position
     *
     * @return current target position
     */
    public double getTarget() {
        return targetPos;
    }

    /**
     * Determines whether you are reasonably close to target position yet
     *
     * @return boolean indicating ^^
     */
    public boolean atTargetPosition() {
        return abs(position - targetPos) < TICK_STOP_PADDING;
    }

    /**
     * Sets direction of motor
     *
     * @param p_direction input direction for motor
     */
    public void setDirection(DcMotorSimple.Direction p_direction) {
        rfMotor.setDirection(p_direction);
    }

    /**
     * Overrides encoder position; artificially sets encoder to 0 or max
     *
     * @param p_position position parameter
     */
    public void setCurrentPosition(double p_position) {
        additionalTicks = p_position - rfMotor.getCurrentPosition();
    }

    public void resetPosition(){
        rfMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Manually set power
     *
     * @param p_power target power
     */
    public void setPower(double p_power) {
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfMotor.setPower(p_power - kP * -RESISTANCE);

    }

    /**
     * Manually set raw power
     *
     * @param p_power target raw power
     */
    public void setRawPower(double p_power) {
        rfMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfMotor.setPower(p_power);
    }

    /**
     * Returns current motor power
     *
     * @return current motor power
     */
    public double getPower() {
        return rfMotor.getPower();
    }

    /**
     * Manually set velocity
     *
     * @param p_velocity target velocity
     */
    public void setVelocity(double p_velocity) {
        rfMotor.setVelocity(p_velocity);
    }

    /**
     * Returns current encoder position + stored additionalTicks for offset
     *
     * @return same as above ^
     */
    public int getCurrentPosition() {
        if(profile==null || profile.isProfileDone(time)){
            position = rfMotor.getCurrentPosition();
        }
        return /*rfMotor.getCurrentPosition()*/(int) position + (int) additionalTicks;
    }

    public double getTarCurPos() {
        return startPosition+direction*(profile.motionProfileTimeToDist(time));
    }
    public double getTargVel(){
        return profile.calculateTargetVelocity(time)*direction;
    }

    public void setMode(DcMotor.RunMode p_runMode) {
        rfMotor.setMode(p_runMode);
        if (rfMotor.getMode() != p_runMode) {
            logger.log("/MotorLogs/RFMotor", rfMotorName + ",setMode(),Setting RunMode: " + p_runMode,
                    true, true);
        }
    }

    /**
     * Returns current motor velocity
     *
     * @return current motor velocity
     */
    public double getVelocity() {
        if(profile==null){
            velocity = rfMotor.getVelocity();
        }
        return velocity;
    }

    public void setTarget(double p_target) {
        targetPos = p_target;
    }

    //Below functions are used for simulation
    double lastUpdateTime = 0.0;

    /**
     * Updates variables in RFPoseSim storage
     *
     * @param p_targetPos   Target position
     * @param p_tickPos     Simulated current tick position of the motor
     * @param p_targetVelo  Target Velocity
     * @param p_targetAccel Target Acceleration
     */
    public void getTargets(double p_targetPos, double p_tickPos, double p_targetVelo, double p_targetAccel) {
        currentPos = p_targetPos;
        currentTickPos = p_tickPos;
        velocity = p_targetVelo;
        currentAcceleration = p_targetAccel;
        lastUpdateTime = time;
    }

    /**
     * Visually updates the simulation
     */
    public void updateSim() {
        Pose2d targetPose = new Pose2d(currentPos, 0, 0);
        lastUpdateTime = time;
        Canvas fieldOverlay = packet.fieldOverlay();
        fieldOverlay.setStrokeWidth(1);
        fieldOverlay.setStroke("#4CAF50");
        DashboardUtil.drawRobot(fieldOverlay, targetPose);
    }

    /**
     * Calculates and updates both the stored variables in RFPoseSim and the visual simulation
     */
    public void update() {
        getTargets(getTargetPosition(BasicRobot.time) * 0.05, getTargetPosition(BasicRobot.time),
                getTargetVelocity(BasicRobot.time), getTargetAcceleration(BasicRobot.time));
        updateSim();
    }

    /**
     * Change the isSim boolean to true or false
     *
     * @param p_IsSim desired value of isSim
     */
    public void setIsSim(boolean p_IsSim) {
        isSim = p_IsSim;
    }

    /**
     * Simplified setPosition() function just for simulation; only gets targets
     *
     * @param p_targetPos target position
     * @param p_curve     desired curve for S-Curve
     * @return target velocity and acceleration at current time
     */
    public double[] setSimPosition(double p_targetPos, double p_curve) {
        if (p_targetPos > maxtickcount) {
            p_targetPos = maxtickcount;
        }
        if (p_targetPos < mintickcount) {
            p_targetPos = mintickcount;
        }
        position = currentPos;
        targetPos = p_targetPos;
        lastTime = time;
        isSim = true;

        return getTargetMotion(p_curve);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior p_bheavior) {
        rfMotor.setZeroPowerBehavior(p_bheavior);
    }
}