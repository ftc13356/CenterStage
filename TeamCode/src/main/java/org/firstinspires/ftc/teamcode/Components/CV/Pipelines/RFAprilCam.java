package org.firstinspires.ftc.teamcode.Components.CV.Pipelines;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.logger;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.PoseStorage.currentPose;

import static java.lang.Math.PI;
import static java.lang.Math.min;
import static java.lang.Math.sqrt;

import android.os.Build;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.apache.commons.math3.analysis.function.Exp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.concurrent.TimeUnit;

/**
 * Warren
 * All operations associated with aprilTag
 */
@Config
public class RFAprilCam {
    public static double X_OFFSET = 0, Y_OFFSET = 0, UPSAMPLE_THRESHOLD=40, NUMBER_OF_SAMPLES=50;
    public static int EXPOSURE_MS=15, GAIN = 5;
    public static double DOWNSAMPLE = 2, UPSAMPLE = 6;
    private AprilTagProcessor aprilTag;
    public RFVisionPortal visionPortal;
    boolean upsample=false;
    ExposureControl exposureControl;
    GainControl gainControl;
    AprilTagLibrary aprilTagGameDatabase;
    private Vector2d[] values = {new Vector2d(0, 0), new Vector2d(0, 0),new Vector2d(0, 0),new Vector2d(0, 0),
            new Vector2d(0, 0),new Vector2d(0, 0)
            ,new Vector2d(0, 0),new Vector2d(3*23.5-1,-1.5*23.5),new Vector2d(3*23.5-1,-1.5*23.5+4.5)
            ,new Vector2d(3*23.5-1,1.5*23.5),new Vector2d( 3*23.5-1,1.5*23.5-4.5)};
    private Pose2d camPoseError = new Pose2d(0,0,0);
    private double poseCount =0;
    private double[][] directions = {{-1, 1}, {-1, 1},{-1, 1},{-1, 1},{-1, 1},{-1, 1}
            ,{-1, 1},{-1, -1},{-1, -1},{-1, 1},{-1, 1}};
    private double[] angle = {-1,-1,-1,-1,-1,-1,
    -1,-1,-1,1,-1};
    /**
     * Initialize apriltag camera
     * Logs that function is called and camera is aprilTag  general surface level
     */
    public RFAprilCam() {
        aprilTagGameDatabase = AprilTagGameDatabase.getCenterStageTagLibrary();
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(false)
                .setDrawCubeProjection(false)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
                .build();
        aprilTag.setPoseSolver(AprilTagProcessor.PoseSolver.OPENCV_IPPE_SQUARE);
        aprilTag.setDecimation((float)UPSAMPLE);

        // Create the WEBCAM vision portal by using a builder.
        visionPortal = new RFVisionPortal.Builder()
                .setCamera(op.hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .enableLiveView(false)
                .setStreamFormat(RFVisionPortal.StreamFormat.MJPEG)
                .setCameraResolution(new Size(640, 480))
                .build();
        visionPortal.stopLiveView();
        while(visionPortal.getCameraState()!= RFVisionPortal.CameraState.STREAMING){
            op.sleep(50);
        }
        exposureControl = visionPortal.getCameraControl(ExposureControl.class);
        exposureControl.setMode(ExposureControl.Mode.Manual);
        exposureControl.setExposure(EXPOSURE_MS, TimeUnit.MILLISECONDS);
        exposureControl.setAePriority(false);
        op.sleep(50);
        gainControl = visionPortal.getCameraControl(GainControl.class);
        GAIN = min(GAIN, gainControl.getMaxGain());
        gainControl.setGain(GAIN);
        op.sleep(50);

    }

    /**
     * Updates stored info to the latest available apriltag data
     * Logs newly calculated position at finest verbosity level
     */
    public void update() {
        ArrayList<AprilTagDetection> detections = aprilTag.getFreshDetections();
        //if close start upsampling
        upsample=false;
        for (AprilTagDetection detection : detections) {
             AprilTagPoseFtc poseFtc= detection.ftcPose;
             double p_x = poseFtc.y, p_y = poseFtc.x;
             var p_ind = detection.id;
             AprilTagMetadata tagData = aprilTagGameDatabase.lookupTag(p_ind);
             VectorF values = tagData.fieldPosition;
             Vector2d pos = new Vector2d(values.get(0),values.get(1));
             var camPose = new Pose2d(pos.plus(new Vector2d(p_x * directions[p_ind][0]+X_OFFSET, p_y * directions[p_ind][1]+Y_OFFSET)),
                     directions[p_ind][0]*poseFtc.yaw*PI/180);
             if(poseFtc.range<UPSAMPLE_THRESHOLD){
                 upsample=true;
                 camPoseError.plus(camPose).minus(currentPose);
                 poseCount++;
             }
             logger.log("/RobotLogs/GeneralRobot", "id: "+ p_ind+ " aprilPos = "+camPose+", dist:"+poseFtc.range+" p_x, p_y: " + p_x +',' +p_y);
        }
        if(upsample && poseCount>NUMBER_OF_SAMPLES){
            logger.log("/RobotLogs/GeneralRobot", "avgAprilPose"+currentPose.plus(camPoseError.div(poseCount)));
            currentPose = currentPose.plus(camPoseError.div(poseCount));
            logger.log("/RobotLogs/GeneralRobot", "avgPose"+currentPose);
            poseCount=0;
            camPoseError=new Pose2d(0,0,0);
        }
    }

    /**
     * Gets the most recently stored camera position
     * @return cameraPosition
     */
    public Pose2d getCamPose(){
        return currentPose.plus(camPoseError.div(poseCount));
    }
}
