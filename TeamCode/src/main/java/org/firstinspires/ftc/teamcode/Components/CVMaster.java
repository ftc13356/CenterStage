package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.dashboard;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.isTeleop;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.SampleDetect.BluePipeline;
import org.firstinspires.ftc.teamcode.Components.SampleDetect.ExcludePipline;
import org.firstinspires.ftc.teamcode.Components.SampleDetect.RedPipeline;
import org.firstinspires.ftc.teamcode.Components.SampleDetect.SampleDetectTest;
import org.firstinspires.ftc.teamcode.Components.SampleDetect.SampleDetectionPipeline;
import org.firstinspires.ftc.teamcode.Components.SampleDetect.SampleDetectionPipelinePNP;
import org.firstinspires.ftc.teamcode.Components.SampleDetect.YellowPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Config
public class CVMaster {
    public static boolean shouldStream = true;
    public static boolean isHsvTest = false;
    OpenCvWebcam webcam;

    ExcludePipline exclude;
    SampleDetectionPipelinePNP hsvTest;
    public CVMaster(){
        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(op.hardwareMap.get(WebcamName.class, "Webcam 2"));
            exclude = new ExcludePipline();
            hsvTest = new SampleDetectionPipelinePNP();
//            if(isTeleop) {
                startStreamin();
//            }
    }
    public double[] getCenter(){
        return exclude.getCenter();
    }
    public void startStreamin(){
        webcam.openCameraDeviceAsync(
                new OpenCvCamera.AsyncCameraOpenListener() {
                    @Override
                    public void onOpened() {
                        /*
                         * Tell the webcam to start streaming images to us! Note that you must make sure
                         * the resolution you specify is supported by the camera. If it is not, an exception
                         * will be thrown.
                         *
                         * Keep in mind that the SDK's UVC driver (what OpenCvWebcam uses under the hood) only
                         * supports streaming from the webcam in the uncompressed YUV image format. This means
                         * that the maximum resolution you can stream at and still get up to 30FPS is 480p (640x480).
                         * Streaming at e.g. 720p will limit you to up to 10FPS and so on and so forth.
                         *
                         * Also, we specify the rotation that the webcam is used in. This is so that the image
                         * from the camera sensor can be rotated such that it is always displayed with the image upright.
                         * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                         * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                         * away from the user.
                         */
                        //            if (isRed&&!isBlue) {
                        //              webcam.setPipeline(openSleevi);
                        //            } else if(isRed&&isBlue){
                        //              webcam.setPipeline(openSleeviy);
                        //            }else if(!isRed&&isBlue){
                        //              webcam.setPipeline(openSleevey);
                        //            }
                        //            else {
                        //              webcam.setPipeline(openSleeve);
                        //            }
                        if(!isHsvTest)
                            webcam.setPipeline(exclude);
                        else
                            webcam.setPipeline(hsvTest);
                        webcam.startStreaming(
                                    640, 480, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);

                        webcam.setViewportRenderer(OpenCvCamera.ViewportRenderer.NATIVE_VIEW);
//                        dashboard.startCameraStream(webcam,5);

                        if(shouldStream)
                            dashboard.startCameraStream(webcam, 10);
                        LOGGER.log("Camera Streaming now!");
                    }

                    @Override
                    public void onError(int errorCode) {
                        /*
                         * This will be called if the camera could not be opened
                         */
                    }
                });
    }
    public int getCurrent(){
        return exclude.getColor();
    }
    public void swapInt(int swap){
        exclude.setColor(swap);
    }
    public void swapNext(){
        int newy = exclude.getColor()+1;
        if (newy > 2) newy = 0;
        exclude.setColor(newy);
    }
    public double getLatency(){
        return .001*webcam.getTotalFrameTimeMs();
    }
    public void swapRed(){

        exclude.setColor(0);
    }
    public void swapBlue(){
        exclude.setColor(1);
    }
    public void swapYellow(){
        exclude.setColor(2);
    }
    public void resetCenter(){
        exclude.resetCenter();
    }
    public void swapTeleRed() {exclude.isBlue = false;
        exclude.setColor(0);}
    public void swapTeleBlue() {exclude.isBlue = true; exclude.setColor(1);}
}
