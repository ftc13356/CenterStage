package org.firstinspires.ftc.teamcode.TeleOp.Outreach;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp()
public class OutreachClawbot extends LinearOpMode {
    private DcMotor frontRight = null;
    private DcMotor frontLeft = null;
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private Servo claw = null;
    boolean openclaw = false;


    @Override
    public void runOpMode() throws InterruptedException {
        frontRight = hardwareMap.get(DcMotor.class, "motorRightFront");
        frontLeft = hardwareMap.get(DcMotor.class, "motorLeftFront");
        backLeft = hardwareMap.get(DcMotor.class, "motorLeftBack");
        backRight = hardwareMap.get(DcMotor.class, "motorRightBack");
        claw = hardwareMap.get(Servo.class, "claw");


        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        openclaw = false;
        waitForStart();


        while(!isStopRequested()&&opModeIsActive()){
            telemetry.update();
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            double strafe = gamepad1.right_stick_x;
            telemetry.addData("strafe", Double.toString(strafe));
            telemetry.addData("drive", Double.toString(drive));
            telemetry.addData("turn", Double.toString(turn));

            frontLeft.setPower(drive+turn+strafe);
            backLeft.setPower(drive-turn+strafe);
            frontRight.setPower(drive-turn-strafe);
            backRight.setPower(drive+turn-strafe);
            if (gamepad1.x && openclaw == true){
                claw.setPosition(0.4);
                openclaw = false;
            } else if (!gamepad1.x && !openclaw){
                claw.setPosition(0);
                openclaw=true;
            }




//            if (drive>0 || drive<0){
//                frontRight.setPower(drive);
//                frontLeft.setPower(drive);
//                backRight.setPower(drive);
//                backLeft.setPower(drive);
//            }
//
//            if (turn>0 || turn<0){
//                frontRight.setPower(turn);
//                backRight.setPower(turn);
//                frontLeft.setPower(-turn);
//                backLeft.setPower(-turn);
//            }
//
//            if (strafe>0 || strafe<0){
//                frontLeft.setPower(strafe);
//                backRight.setPower(strafe);
//                frontRight.setPower(-strafe);
//                backLeft.setPower(-strafe);
//            }



        }

    }
}

