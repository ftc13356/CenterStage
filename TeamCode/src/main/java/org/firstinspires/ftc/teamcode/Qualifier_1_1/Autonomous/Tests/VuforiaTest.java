/**
 * Vuforia Test. Inits Vuforia and it
 * displays the target it sees, the location
 * of the phone/robot relative to the target,
 * and the angle/rotation of the phone.
 *
 * @author  Aamod
 * @version 1.0
 * @since   2020-July-10
 * @status: Fully working
 */

package org.firstinspires.ftc.teamcode.Qualifier_1_1.Autonomous.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Qualifier_1_1.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Qualifier_1_1.Components.Navigations.VuforiaWebcam;
import org.firstinspires.ftc.teamcode.Qualifier_1_1.Robot;

@Autonomous(name = "VuforiaTest")
@Disabled
public class VuforiaTest extends LinearOpMode {


    @Override
    public void runOpMode() {

        Robot robot = new Robot(this, BasicChassis.ChassisType.ENCODER);
        ElapsedTime runtime = new ElapsedTime();

        //AamodVuforia vuforiaB = new AamodVuforia(this, VuforiaLocalizer.CameraDirection.BACK);
        //AamodVuforia vuforiaF = new AamodVuforia(this, VuforiaLocalizer.CameraDirection.FRONT);
        VuforiaWebcam vuforiaWebcam = new VuforiaWebcam(this);

        vuforiaWebcam.init(this);
        vuforiaWebcam.start();

        waitForStart();

        while (opModeIsActive()) {
            //Webcam

//            while (opModeIsActive()) {
//
//                //Webcam
//
//                double x = vuforiaWebcam.getVuforiaX();
//                double y = vuforiaWebcam.getVuforiaY();
//                double angle = vuforiaWebcam.getVuforiaAngle();
//
//                telemetry.addData("Back X", x);
//                telemetry.addData("Back Y", y);
//                telemetry.addData("Back Angle", "%.2f, %.2f", angle, angle + 90);
//                telemetry.addData("Back Target", vuforiaWebcam.getVuforiaTrackable());
//                telemetry.update();
//
//                //Back
//                telemetry.addData("Back X", vuforiaB.getVuforiaX());
//                telemetry.addData("Back Y", vuforiaB.getVuforiaY());
//                telemetry.addData("Back Angle", vuforiaB.getVuforiaAngle());
//                telemetry.addData("Back Target", vuforiaB.getVuforiaTrackable());
//                sleep(500);
//                //Front
//                telemetry.addData("Front X", vuforiaF.getVuforiaX());
//                telemetry.addData("Front Y", vuforiaF.getVuforiaY());
//                telemetry.addData("Front Angle", vuforiaF.getVuforiaAngle());
//                telemetry.addData("Front Target", vuforiaF.getVuforiaTrackable());
//                telemetry.update();
//            }
//            vuforiaWebcam.interrupt();
        }
    }
}