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
    public static double R_THRESH=.04, G_THRESH=.06, B_THRESH=.04, D_THRESH = .4;
    int lastColor = 1;

    public Colorsens(){
        color = op.hardwareMap.get(RevColorSensorV3.class, "color");
    }
    public int getColor(){
        NormalizedRGBA colo = color.getNormalizedColors();
        double dist = color.getDistance(DistanceUnit.INCH);
        packet.put("R", colo.red);
        packet.put("G", colo.green);
        packet.put("B", colo.blue);
        packet.put("D", dist);
        if(dist<D_THRESH){
            if(colo.red>R_THRESH && colo.green > G_THRESH && colo.blue > B_THRESH){
                packet.put("color", "white");
                lastColor=1;
                return 1;
            }
            else {
                packet.put("color", "yellow");
                lastColor=0;
                return 0;
            }
        }
        return 2;
    }
}
