package org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.SparkFunOTOSCorrected;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;

/**
 * This is the OTOSLocalizer class. This class extends the Localizer superclass and is a
 * localizer that uses the SparkFun OTOS. The diagram below, which is modified from
 * Road Runner, shows a typical set up.
 *
 * The view is from the top of the robot looking downwards.
 *
 * left on robot is the y positive direction
 *
 * forward on robot is the x positive direction
 *
 *    /--------------\
 *    |     ____     |
 *    |     ----     |
 *    | ||        || |
 *    | ||        || |  ----> left (y positive)
 *    |              |
 *    |              |
 *    \--------------/
 *           |
 *           |
 *           V
 *    forward (x positive)
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 7/20/2024
 */
@Config
public class OTOSLocalizer extends Localizer {
    private HardwareMap hardwareMap;
    private Pose startPose;
    private SparkFunOTOS otos, otos2;
    private SparkFunOTOS.Pose2D otosPose, otosPose2;
    private SparkFunOTOS.Pose2D otosVel, otosVel2;
    private SparkFunOTOS.Pose2D otosAcc, otosAcc2;
    private int greater1Count = 0, greater2Count = 0;
    private double previousHeading;
    private double totalHeading;
    public static double MULT1 = .99730660924, MULT2 = .98714377, HMULT1 = 0.98450498725, HMULT2 = .99705,  WEIGHT = 0.5;

    /**
     * This creates a new OTOSLocalizer from a HardwareMap, with a starting Pose at (0,0)
     * feacing 0 heading.
     *
     * @param map the HardwareMap
     */
    public OTOSLocalizer(HardwareMap map) {
        this(map, new Pose());
    }

    /**
     * This creates a new OTOSLocalizer from a HardwareMap and a Pose, with the Pose
     * specifying the starting pose of the localizer.
     *
     * @param map the HardwareMap
     * @param setStartPose the Pose to start from
     */
    public OTOSLocalizer(HardwareMap map, Pose setStartPose) {
        hardwareMap = map;

        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos2 = hardwareMap.get(SparkFunOTOS.class,"otos2");

        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.RADIANS);
        otos2.setLinearUnit(DistanceUnit.INCH);
        otos2.setAngularUnit(AngleUnit.RADIANS);
//        otos.setSignalProcessConfig(new SparkFunOTOS.SignalProcessConfig((byte) 0x0D));

        // For the OTOS, left/right is the y axis a nd forward/backward is the x axis, with left being
        // positive y and forward being positive x. PI/2 radians is facing forward, and clockwise
        // rotation is negative rotation.
        otos2.setOffset(new SparkFunOTOS.Pose2D(7.25,-3,Math.PI/2));
        otos.setOffset(new SparkFunOTOS.Pose2D(7.25,3,-Math.PI/2));


        otos.setLinearScalar(1);
        otos.setAngularScalar(HMULT1);

        otos.calibrateImu();
        otos.resetTracking();
        otos2.setLinearScalar(1);
        otos2.setAngularScalar(HMULT2);

        otos2.calibrateImu();
        otos2.resetTracking();

        setStartPose(setStartPose);
        otosPose = new SparkFunOTOS.Pose2D();
        otosVel = new SparkFunOTOS.Pose2D();
        otosAcc = new SparkFunOTOS.Pose2D();
        otosPose2 = new SparkFunOTOS.Pose2D();
        otosVel2 = new SparkFunOTOS.Pose2D();
        otosAcc2 = new SparkFunOTOS.Pose2D();
        totalHeading = 0;
        previousHeading = startPose.getHeading();
        greater2Count=0;
        greater1Count=0;
        if(!isTeleop)
            WEIGHT = 0.5;
        resetOTOS();
    }

    /**
     * This returns the current pose estimate.
     *
     * @return returns the current pose estimate as a Pose
     */
    @Override
    public Pose getPose() {
        return MathFunctions.addPoses(startPose, new Pose(otosPose.x, otosPose.y, otosPose.h));
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Pose
     */
    @Override
    public Pose getVelocity() {
        return new Pose(otosVel.x, otosVel.y, otosVel.h);
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Vector
     */
    @Override
    public Vector getVelocityVector() {
        return getVelocity().getVector();
    }

    /**
     * This sets the start pose. Changing the start pose should move the robot as if all its
     * previous movements were displacing it from its new start pose.
     *
     * @param setStart the new start pose
     */
    @Override
    public void setStartPose(Pose setStart) {
        startPose = setStart;
    }

    /**
     * This sets the current pose estimate. Changing this should just change the robot's current
     * pose estimate, not anything to do with the start pose.
     *
     * @param setPose the new current pose estimate
     */
    @Override
    public void setPose(Pose setPose) {
        resetOTOS();
        Pose setOTOSPose = MathFunctions.subtractPoses(setPose, startPose);
        otos.setPosition(new SparkFunOTOS.Pose2D(setOTOSPose.getX(), setOTOSPose.getY(), setOTOSPose.getHeading()));
    }

    /**
     * This updates the total heading of the robot. The OTOS handles all other updates itself.
     */
    @Override
    public void update() {
        if(WEIGHT!=0)
            otos.getPosVelAcc(otosPose,otosVel,otosAcc);
        if(WEIGHT!=1)
            otos2.getPosVelAcc(otosPose2,otosVel2,otosAcc2);
        Vector2d vel0 = new Vector2d(otosVel.x, 0);
        Vector2d vel1 = new Vector2d(otosVel2.x, 0);

        if(vel0.norm()*MULT1>5 && vel0.norm()*MULT1>1.05*vel1.norm()*MULT2){
            greater1Count++;
            greater2Count=0;
        } else if(vel1.norm()*MULT2>5 && vel1.norm()*MULT2>1.05*vel0.norm()*MULT1){
            greater2Count++;
            greater1Count=0;
        }
        else{
            greater1Count=0;
            greater2Count = 0;
        }
        if(greater1Count>30 && WEIGHT==0.5){
            WEIGHT=1;
        }else if(greater2Count>30&& WEIGHT==0.5){
            WEIGHT=0;
        }
        packet.put("x0", otosPose.x);
        packet.put("x1",otosPose2.x);
        otosPose.set(new SparkFunOTOS.Pose2D(WEIGHT*otosPose.x*MULT1+(1-WEIGHT)*otosPose2.x*MULT2,WEIGHT*otosPose.y*MULT1+(1-WEIGHT)*otosPose2.y*MULT2,0.5*otosPose.h+0.5*otosPose2.h));
        otosVel.set(new SparkFunOTOS.Pose2D(WEIGHT*otosVel.x*MULT1+(1-WEIGHT)*otosVel2.x*MULT2,WEIGHT*otosVel.y*MULT1+(1-WEIGHT)*otosVel2.y*MULT2,0.5*otosVel.h+0.5*otosVel2.h));
        otosAcc.set(new SparkFunOTOS.Pose2D(WEIGHT*otosAcc.x*MULT1+(1-WEIGHT)*otosAcc2.x*MULT2,WEIGHT*otosAcc.y*MULT1+(1-WEIGHT)*otosAcc2.y*MULT2,0.5*otosAcc.h+0.5*otosAcc2.h));
        totalHeading += MathFunctions.getSmallestAngleDifference(otosPose.h, previousHeading);
        previousHeading = otosPose.h;
        packet.put("vel0",vel0.norm()*MULT1);
        packet.put("vel1",vel1.norm()*MULT2);
        packet.put("velRatio", vel0.norm()*MULT1/vel1.norm()*MULT2);
        packet.put("greater0", greater1Count);
        packet.put("greater1",greater2Count);

    }

    /**
     * This resets the OTOS.
     */
    public void resetOTOS() {
        otos.resetTracking();
        otos2.resetTracking();
    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    public double getTotalHeading() {
        return totalHeading;
    }

    /**
     * This returns the multiplier applied to forward movement measurement to convert from OTOS
     * ticks to inches. For the OTOS, this value is the same as the lateral multiplier.
     * This is found empirically through a tuner.
     *
     * @return returns the forward ticks to inches multiplier
     */
    public double getForwardMultiplier() {
        return otos.getLinearScalar();
    }

    /**
     * This returns the multiplier applied to lateral/strafe movement measurement to convert from
     * OTOS ticks to inches. For the OTOS, this value is the same as the forward multiplier.
     * This is found empirically through a tuner.
     *
     * @return returns the lateral/strafe ticks to inches multiplier
     */
    public double getLateralMultiplier() {
        return otos.getLinearScalar();
    }

    /**
     * This returns the multiplier applied to turning movement measurement to convert from OTOS ticks
     * to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    public double getTurningMultiplier() {
        return otos.getAngularScalar();
    }

    /**
     * This does nothing since this localizer does not use the IMU.
     */
    public void resetIMU() {
    }
}