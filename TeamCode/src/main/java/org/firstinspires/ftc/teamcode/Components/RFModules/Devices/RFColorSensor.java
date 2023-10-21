package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.apache.commons.math3.stat.StatUtils.min;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.LOGGER;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.teamcode.Components.RFModules.System.RFLogger;

import java.text.DecimalFormat;

/**
 * Harry
 * Class to contain all RFColorSensor functions
 */
public class RFColorSensor {
    NormalizedColorSensor colorSensor;
    private double white = 156, purple = 214, green = 127, yellow = 81;
    private float HSV[] = {0,0,0};
    private float tempHSV[] = {0,0,0};
    private double errorWt = 0, errorPt = 0, errorYt = 0, errorGt = 0;
    private String result = "NONE";
    private String tempResult = "NONE";

    /**
     * constructor for rfcolorsensor, logs to general with CONFIG severity
     * @param p_deviceName
     */
    public RFColorSensor(String p_deviceName){
        colorSensor = op.hardwareMap.get(NormalizedColorSensor.class, p_deviceName);
        LOGGER.log(RFLogger.Severity.CONFIG, "RFColorSensor initialized, device name = " + p_deviceName);
        LOGGER.log(RFLogger.Severity.CONFIG, "Hue values: WHITE = 156 | PURPLE = 214 | GREEN = 127 | YELLOW = 81");
    }

    public float[] getHSV(){
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), HSV);
        if(HSV != tempHSV){
            tempHSV = HSV;
            LOGGER.log(RFLogger.Severity.INFO, "HSV readings: H: " + tempHSV[0] + " | S: " + tempHSV[1] + " | V: " + tempHSV[2]);
        }
        return HSV;

    }

    public String getColor(){
        double errorW = 0, errorP = 0, errorG = 0, errorY = 0;
        float[] hsv = getHSV();
        errorW = Math.abs(hsv[0]-white); errorP = Math.abs(hsv[0]-purple); errorG = Math.abs(hsv[0]-green); errorY = Math.abs(hsv[0]-yellow);
        if(errorW != errorWt || errorP != errorPt || errorG != errorGt || errorY != errorYt){
            errorWt = errorW; errorPt = errorP; errorGt = errorG; errorYt = errorY;
            LOGGER.log(RFLogger.Severity.INFO, "Hue difference: W " + errorWt + " | P " + errorPt + " | G " + errorGt + " | Y " + errorYt);
        }

        if(Math.min(Math.min(errorW, errorP),Math.min(errorY, errorG)) == errorW){
            result = "WHITE";
        }
        if(Math.min(Math.min(errorW, errorP),Math.min(errorY, errorG)) == errorP){
            result = "PURPLE";
        }
        if(Math.min(Math.min(errorW, errorP),Math.min(errorY, errorG)) == errorY){
            result = "YELLOW";
        }
        if(Math.min(Math.min(errorW, errorP),Math.min(errorY, errorG)) == errorG){
            result = "GREEN";
        }
        if(!result.equals(tempResult)){
            tempResult = result;
            LOGGER.log(RFLogger.Severity.INFO, "Predicted Color: " + tempResult);
        }
        return result;
    }
}