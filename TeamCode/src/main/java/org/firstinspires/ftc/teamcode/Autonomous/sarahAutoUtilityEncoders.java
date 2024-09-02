package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class sarahAutoUtilityEncoder{
    public DcMotor backLeft;
    public DcMotor backRight;
    private HardwareMap hardwareMap;
    public double ticks;
    public LinearOpMode opMode;
    private ElapsedTime runtime = new ElapsedTime();

    //put this gross ass code into some methods pls god
    public sarahAutoUtilityEncoder(LinearOpMode op) {
        hardwareMap = op.hardwareMap;
        opMode = op;
        backLeft = hardwareMap.get(DcMotor.class, "motorLeftBack");
        setDirectionAndMode(backLeft, "reverse", "usingEncoder");

        backRight = hardwareMap.get(DcMotor.class, "motorRightBack");
        setDirectionAndMode(backLeft, "forward", "usingEncoder");
    }
    public void moveForward(double distance){
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        ticks = getTicks(384.5, distance, 2);
        backLeft.setTargetPosition((int)ticks);
        backRight.setTargetPosition((int)ticks);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(.4,.4);
        while (opMode.opModeIsActive() &&
                (runtime.seconds() < 3) && //pulled this numebr otu of myass
                (backLeft.isBusy() && backRight.isBusy())) {
            opMode.sleep(50);
        }
        setPower(0,0);
    }
    public void turn(double degrees){ //naur
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setTargetPosition((int)384.5/2); //idec naymore ong bru
        backRight.setTargetPosition((int)384.5/2);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        setPower(.15,.15);
    }
    public void setDirectionAndMode (DcMotor motor, String direction, String mode){
        if(direction == "forward"){
            motor.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            motor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        if(mode == "usingEncoder"){
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } else if(mode =="toPosition") {
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }
    public void setPower(double powerLeft, double powerRight){
        backLeft.setPower(powerLeft);
        backRight.setPower(powerRight);
    }
    public double getTicks(double ticks, double distance, double wheelRadius) {
        if(ticks == 0.0) {
            ticks = 384.5;
        }
        double circum = 2 * wheelRadius * 3.14159;
        double turns = 10/circum;
        return turns * ticks; //help i can't do math
    }
}
