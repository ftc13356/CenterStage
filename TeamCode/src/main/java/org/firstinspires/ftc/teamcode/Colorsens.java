package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.op;
import static org.firstinspires.ftc.teamcode.Robots.BasicRobot.packet;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Colorsens {
    RevColorSensorV3 color;
    public static int WR_THRESH=200, WG_THRESH=200, WB_THRESH=200, YR_THRESH=200, YB_THRESH=200, YG_THRESH=100, D_THRESH = 1;

    public Colorsens(){
        color = op.hardwareMap.get(RevColorSensorV3.class, "color");
    }
    public int getColor(){
        NormalizedRGBA colo = color.getNormalizedColors();
        double dist = color.getDistance(DistanceUnit.INCH);
        packet.put("R", colo.red);
        packet.put("G", colo.red);
        packet.put("B", colo.red);
        packet.put("D", dist);
        if(dist<D_THRESH){
            if(colo.red>WR_THRESH && colo.green > WG_THRESH && colo.blue > WB_THRESH){
                return 2;
            }
            else if(colo.red>YR_THRESH && colo.blue > YB_THRESH){
                return 1;
            }
        }
        return 0;
    }
}
