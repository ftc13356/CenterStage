package org.firstinspires.ftc.teamcode.Robots;


import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFGamepad;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.Logger;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.Queuer;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;

import java.util.List;

/**
 * Warren Zhou
 * 9/1
 * Basic robot with basic features that all should have
 */

public class BasicRobot{
    public static Logger logger;
    public static RFLogger LOGGER;
    public static LinearOpMode op = null;
    public Queuer queuer;
    public static boolean isTeleop;
    public static FtcDashboard dashboard;
    public static double time= 0.0, voltage = 12;
    public static int loops = 0;
    public static VoltageSensor voltageSensor;
    public static TelemetryPacket packet;
    public static RFGamepad gampad;
    public static final boolean isSim = false;
    public double lastLoopTime = -100;

    /**
     * instantiates basic robot
     * Logs that function is called to general surface
     * @param opMode linearOpMode, auto or teleOp class
     * @param p_isTeleop is it teleOp
     */

    public BasicRobot(LinearOpMode opMode, boolean p_isTeleop, boolean isFlipped){
        op = opMode;
        LOGGER = new RFLogger("Robot");
        LOGGER.setLogLevel(RFLogger.Severity.ALL);
        LOGGER.log("Creating Robot!");
        logger = new Logger();
        logger.createFile("/RobotLogs/GeneralRobot", "Runtime    Component               " +
                "Function                        Action");
        logger.createFile("gamepad", "Value Name Time");
        dashboard = FtcDashboard.getInstance();
        queuer = new Queuer();
        isTeleop = p_isTeleop;
        op.telemetry = new MultipleTelemetry(op.telemetry, FtcDashboard.getInstance().getTelemetry());
        if(!isSim) {
            voltageSensor = op.hardwareMap.voltageSensor.iterator().next();
        }
        dashboard.setTelemetryTransmissionInterval(25);
        packet=new TelemetryPacket();
        gampad = new RFGamepad();
        for(LynxModule module: op.hardwareMap.getAll(LynxModule.class))
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        voltage = voltageSensor.getVoltage();
        lastLoopTime = -100;
    }
    public BasicRobot(LinearOpMode opMode, boolean p_isTeleop){
        this(opMode, p_isTeleop, false);
    }



    /**
     * updates all system files
     * logs that this function is being called to general finest
     */

    public void update(){
        loops++;
        time = op.getRuntime();
        if(abs(time-lastLoopTime)>1){
            packet.put("loopTime", loops/abs(time-lastLoopTime));
            lastLoopTime = time;
            loops=0;
        }
        dashboard.sendTelemetryPacket(packet);
        packet = new TelemetryPacket();
        packet.clearLines();
        op.telemetry.update();
        packet.put("time", time);
        for(LynxModule module: op.hardwareMap.getAll(LynxModule.class))
            module.clearBulkCache();
    }

    /**
     * resets the queuer
     * logs that this function is being called to general surface
     */
    public void resetQueuer() {
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("queuer reset");
        queuer.reset();
    }


    /**
     * gets the current voltage
     * logs that this function is being called and the currentVoltage to general surface
     * @return the voltage
     */
    public double getVoltage(){
        double voltage = voltageSensor.getVoltage();
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("voltage = "+voltage);
        return voltage;
    }
}
