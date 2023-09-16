package org.firstinspires.ftc.teamcode.Components.RFModules.Devices;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;

import com.qualcomm.hardware.rev.RevColorSensorV3;

public class RFColorSensor {
    private RevColorSensorV3 colorSensor;

    public RFColorSensor(){
        colorSensor = op.hardwareMap.get(RevColorSensorV3.class, "colorSensor");
    }

    public int getColor(String p_color){
        int colorValue = 0;
        if(p_color.equals("red")){
            colorValue = colorSensor.red();
        }
        if(p_color.equals("green")){
            colorValue = colorSensor.green();
        }
        if(p_color.equals("blue")){
            colorValue = colorSensor.blue();
        }
        return colorValue;
    }
}
