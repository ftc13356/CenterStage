package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.dashboard;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

public class CapCam {
    OpenCvWebcam webcam;
    CapCamPipline pipe;

    boolean[] current = {false, false, false};
    boolean isPnp = true;
    public CapCam(){
        webcam = OpenCvCameraFactory.getInstance()
                .createWebcam(op.hardwareMap.get(WebcamName.class, "Webcam 2"));
       pipe = new CapCamPipline();
    }
    public double[] getCenter(){
        return pipe.getCenter();
    }
    public void setTrue(int i){
        for(int j =0;j<3;j++)
            current[j] = false;
        current[i]=true;
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
                        if(isPnp) {
                            webcam.setPipeline(pipe);
                        }
                        webcam.startStreaming(
                                640, 480, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);

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
        return pipe.getColor();
    }
    public void swapInt(int swap){
        pipe.setColor(swap);
    }
    public void swapNext(){
        int newy = pipe.getColor()+1;
        if (newy > 2) newy = 0;
        pipe.setColor(newy);
    }
    public void swapRed(){
        pipe.setColor(0);
    }
    public void swapBlue(){
        pipe.setColor(1);
    }
    public void swapYellow(){
        pipe.setColor(2);
    }
    public void resetCenter(){
        pipe.resetCenter();
    }
}