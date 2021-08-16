package org.firstinspires.ftc.teamcode.Misc;

public class MathFunctions {

    //region translate ticks and cm
    public static double ticksToCMs(double ticks, double wheelCircumference, double ticksPerRot){
        double cmPerTick = wheelCircumference/ticksPerRot;
        return cmPerTick*ticks;
    }
    public static double centimetersToTicks(double cm, double wheelCircumference, double ticksPerRot){
        double ticksPerCM = ticksPerRot/wheelCircumference;
        return ticksPerCM*cm;
    }
    //endregion

    //region angles
    public static double clampAngleDegrees(double angle){
        if(angle < 0)
            angle %= -180;
        else
            angle %= 180;

        return angle;
    }
    //endregion

    public static double absXOverX(double x){
        return x == 0 ? 0 : x/Math.abs(x);
    }
    public static double clamp(double x, double min, double max){
        return Math.max(min, Math.min(x, max));
    }
}

