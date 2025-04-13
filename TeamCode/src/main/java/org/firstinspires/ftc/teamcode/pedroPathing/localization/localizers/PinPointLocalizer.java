package org.firstinspires.ftc.teamcode.pedroPathing.localization.localizers;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Localizer;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Vector;
import org.firstinspires.ftc.teamcode.pedroPathing.util.NanoTimer;


import java.util.Objects;

@Config
public class PinPointLocalizer extends Localizer {
    private HardwareMap hardwareMap;
    private GoBildaPinpointDriver odo;
    private double previousHeading;
    private double totalHeading;
    private Pose startPose;
    private long deltaTimeNano;
    private NanoTimer timer;
    private Pose currentVelocity;
    private Pose pinpointPose;
    private boolean pinpointCooked = false;
    public static boolean reverseX = false, reversY = true;

    /**
     * This creates a new PinpointLocalizer from a HardwareMap, with a starting Pose at (0,0)
     * facing 0 heading.
     *
     * @param map the HardwareMap
     */
    public PinPointLocalizer(HardwareMap map){ this(map, new Pose());}

    /**
     * This creates a new PinpointLocalizer from a HardwareMap and a Pose, with the Pose
     * specifying the starting pose of the localizer.
     *
     * @param map the HardwareMap
     * @param setStartPose the Pose to start from
     */
    public PinPointLocalizer(HardwareMap map, Pose setStartPose){
        hardwareMap = map;

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        setOffsets(2, -0.5, DistanceUnit.INCH);

//        if(useYawScalar) {
//            odo.setYawScalar(yawScalar);
//        }
//
//        if(useCustomEncoderResolution) {
//            odo.setEncoderResolution(customEncoderResolution);
//        } else {
            odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        }

        GoBildaPinpointDriver.EncoderDirection xDir = GoBildaPinpointDriver.EncoderDirection.FORWARD, yDir = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        if (reverseX){
            xDir = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        }
        if (reversY){
            yDir = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        }
        odo.setEncoderDirections(xDir,yDir);

        resetPinpoint();

        setStartPose(setStartPose);
        totalHeading = 0;
        timer = new NanoTimer();
        pinpointPose = startPose;
        currentVelocity = new Pose();
        deltaTimeNano = 1;
        previousHeading = setStartPose.getHeading();

    }

    /**
     * This returns the current pose estimate.
     *
     * @return returns the current pose estimate as a Pose
     */
    @Override
    public Pose getPose() {
        return pinpointPose.copy();
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Pose
     */
    @Override
    public Pose getVelocity() {
        return currentVelocity.copy();
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Vector
     */
    @Override
    public Vector getVelocityVector() {
        return currentVelocity.getVector();
    }

    /**
     * This sets the start pose. Since nobody should be using this after the robot has begun moving,
     * and due to issues with the PinpointLocalizer, this is functionally the same as setPose(Pose).
     *
     * @param setStart the new start pose
     */
    @Override
    public void setStartPose(Pose setStart) {
        if (!Objects.equals(startPose, new Pose()) && startPose != null) {
            Pose currentPose = MathFunctions.subtractPoses(MathFunctions.rotatePose(pinpointPose, -startPose.getHeading(), false), startPose);
            setPose(MathFunctions.addPoses(setStart, MathFunctions.rotatePose(currentPose, setStart.getHeading(), false)));
        } else {
            setPose(setStart);
        }

        this.startPose = setStart;
    }

    /**
     * This sets the current pose estimate. Changing this should just change the robot's current
     * pose estimate, not anything to do with the start pose.
     *
     * @param setPose the new current pose estimate
     */
    @Override
    public void setPose(Pose setPose) {
        odo.setPosition(new Pose(setPose.getX(), setPose.getY(), setPose.getHeading()));
        pinpointPose = setPose;
        previousHeading = setPose.getHeading();
    }

    /**
     * This updates the total heading of the robot. The Pinpoint handles all other updates itself.
     */
    @Override
    public void update() {
        deltaTimeNano = timer.getElapsedTime();
        timer.resetTimer();
        odo.update();
        Pose currentPinpointPose = getPoseEstimate(odo.getPosition(), pinpointPose, deltaTimeNano);
        totalHeading += MathFunctions.getSmallestAngleDifference(currentPinpointPose.getHeading(), previousHeading);
        previousHeading = currentPinpointPose.getHeading();
        Pose deltaPose = MathFunctions.subtractPoses(currentPinpointPose, pinpointPose);
        currentVelocity = new Pose(deltaPose.getX() / (deltaTimeNano / Math.pow(10.0, 9)), deltaPose.getY() / (deltaTimeNano / Math.pow(10.0, 9)), deltaPose.getHeading() / (deltaTimeNano / Math.pow(10.0, 9)));
        pinpointPose = currentPinpointPose;
    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    @Override
    public double getTotalHeading() {
        return totalHeading;
    }

    /**
     * This returns the Y encoder value as none of the odometry tuners are required for this localizer
     * @return returns the Y encoder value
     */
    @Override
    public double getForwardMultiplier() {
        return odo.getEncoderY();
    }

    /**
     * This returns the X encoder value as none of the odometry tuners are required for this localizer
     * @return returns the X encoder value
     */
    @Override
    public double getLateralMultiplier() {
        return odo.getEncoderX();
    }

    /**
     * This returns either the factory tuned yaw scalar or the yaw scalar tuned by yourself.
     * @return returns the yaw scalar
     */
    @Override
    public double getTurningMultiplier() {
        return odo.getYawScalar();
    }

    /**
     * This sets the offsets and converts inches to millimeters
     * @param xOffset How far to the side from the center of the robot is the x-pod? Use positive values if it's to the left and negative if it's to the right.
     * @param yOffset How far forward from the center of the robot is the y-pod? Use positive values if it's forward and negative if it's to the back.
     * @param unit The units that the measurements are given in
     */
    private void setOffsets(double xOffset, double yOffset, DistanceUnit unit) {
        odo.setOffsets(unit.toMm(xOffset), unit.toMm(yOffset));
    }

    /**
     * This resets the IMU. Does not change heading estimation.
     */
    @Override
    public void resetIMU() {
        odo.recalibrateIMU();
    }

    /**
     * This resets the pinpoint.
     */
    private void resetPinpoint() {
        odo.resetPosAndIMU();

        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

    private Pose getPoseEstimate(Pose pinpointEstimate, Pose currentPose, long deltaTime) {
        double x;
        double y;
        double heading;

        pinpointCooked = false;

        if (!Double.isNaN(pinpointEstimate.getX())) {
            x = pinpointEstimate.getX();
        } else {
            x = currentPose.getX() + currentVelocity.getX() * deltaTime / Math.pow(10, 9);
            pinpointCooked = true;
        }

        if (!Double.isNaN(pinpointEstimate.getY())) {
            y = pinpointEstimate.getY();
        } else {
            y = currentPose.getY() + currentVelocity.getY() * deltaTime / Math.pow(10, 9);
            pinpointCooked = true;
        }

        if (!Double.isNaN(pinpointEstimate.getHeading())) {
            heading = pinpointEstimate.getHeading();
        } else {
            heading = currentPose.getHeading() + currentVelocity.getHeading() * deltaTime / Math.pow(10, 9);
            pinpointCooked = true;
        }

        return new Pose(x, y, heading);
    }

    /**
     * This returns whether if any component of robot's position is NaN.
     *
     * @return returns whether the robot's position is NaN
     */
    public boolean isNAN() {
        return pinpointCooked;
    }


    public Vector2d getRotVelocity(){
        Vector2d vec = new Vector2d(odo.getVelX(), odo.getVelY());
        vec.rotated(-odo.getHeading());
        return vec;
    }

}