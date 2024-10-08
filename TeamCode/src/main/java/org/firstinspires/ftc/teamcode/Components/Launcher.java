package org.firstinspires.ftc.teamcode.Components;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;
import org.firstinspires.ftc.teamcode.Robots.BasicRobot;

/**
 * Warren
 * Launcher class, shoot plane, pls zone 1
 */
@Config
public class Launcher extends RFServo {
    public static double SHOOT_POSITION = 0.1, OTHER_POSITION = 1;
    boolean isLoaded = false;

    /**
     * Initializes servo hardware. Loads servo. Sets initial state. Logs these three to general surface level
     */
    public Launcher(){
        super("launcherServo", 1.0);
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("initializing hardware, setting position to OTHER_POSITION : " +OTHER_POSITION);
        super.setPosition(SHOOT_POSITION);
        isLoaded=true;
        super.setLastTime(-100);    }

    /**
     * Shoots the plane, changes state. Logs these two to general surface level
     */
    public void shoot(){
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("shooting, setting position to SHOOT_POSITION : " + SHOOT_POSITION);
        super.superSetPosition(OTHER_POSITION);
        isLoaded = false;
    }

    /**
     * Loads the launcher, changes state. Logs these two to general surface level
     */
    public void load(){
        LOGGER.setLogLevel(RFLogger.Severity.INFO);
        LOGGER.log("loading, setting position to OTHER_POSITIOn : " + OTHER_POSITION);
        super.superSetPosition(SHOOT_POSITION);
        isLoaded = true;
    }
    public boolean getLoaded(){
        return isLoaded;
    }
}
