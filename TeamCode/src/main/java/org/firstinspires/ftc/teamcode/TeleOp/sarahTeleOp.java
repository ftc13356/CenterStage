package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp()
public class sarahTeleOp extends LinearOpMode {
    private DcMotor backRight = null;
    private DcMotor backLeft = null;
    private Servo claw = null; //idk the class but whatever

    @Override
    public void runOpMode() throws InterruptedException {
        backLeft = hardwareMap.get(DcMotor.class, "motorLeftBack");
        backRight = hardwareMap.get(DcMotor.class, "motorRightBack");
        claw = hardwareMap.get(Servo.class, "servoFront");

        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while(!isStopRequested()&&opModeIsActive()){
            telemetry.update();
            double drive = -gamepad1.left_stick_y * 0.7;
            double turn = gamepad1.left_stick_x * 0.8;
            telemetry.addData("drive", Double.toString(drive));
            telemetry.addData("turn", Double.toString(turn));

            backLeft.setPower(drive+turn);
            backRight.setPower(drive-turn);

            if(gamepad1.a) {
                claw.setPosition(1);
            } else {
                claw.setPosition(0);
            }
        }

    }
}
