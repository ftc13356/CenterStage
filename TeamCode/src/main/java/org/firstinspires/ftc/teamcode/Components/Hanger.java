package org.firstinspires.ftc.teamcode.Components;


import static java.lang.Double.min;
import static java.lang.Math.abs;
import static java.lang.Math.signum;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;

public class Hanger extends RFMotor {
    double permaPower = 0.0;
    public Hanger(){
        super("hangerMotor", true);
    }
    public void setPermaPower(double p_permaPower){
        permaPower=signum(p_permaPower)*min(abs(p_permaPower), 0.3);
    }
    @Override
    public void setPower(double p_power){
        setRawPower(p_power+permaPower);
    }
}
