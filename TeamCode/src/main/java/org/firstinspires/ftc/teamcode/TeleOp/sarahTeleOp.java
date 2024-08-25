package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp()
public class sarahTeleOp extends LinearOpMode {
    private DcMotor backRight = null;
    private DcMotor backLeft = null;

    @Override
    public void runOpMode() throws InterruptedException {
        backLeft = hardwareMap.get(DcMotor.class, "motorLeftBack");
        backRight = hardwareMap.get(DcMotor.class, "motorRightBack");

        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        while(!isStopRequested()&&opModeIsActive()){
            telemetry.update();
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.left_stick_x;
            telemetry.addData("drive", Double.toString(drive));
            telemetry.addData("turn", Double.toString(turn));

            backLeft.setPower(drive-turn);
            backRight.setPower(drive+turn);
        }

    }
}
