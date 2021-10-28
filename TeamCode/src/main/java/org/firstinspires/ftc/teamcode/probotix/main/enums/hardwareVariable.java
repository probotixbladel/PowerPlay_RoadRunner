package org.firstinspires.ftc.teamcode.probotix.main.enums;

public enum hardwareVariable {

    /*
     * This class is created for ease of use with multiple robots
     * By making multiple instances of this enum, robot variables can be easily used across all opmodes
     * even using different robot variables in different opmodes is possible
     * all information in this class is robot specific and if used on another robot, it won't work properly
     * */

    // todo edit these values according to the one of the current robot

    //Hier moet je de verschillende waardes van de robot invoeren die de berekening die autohelper nodig heeft om
    //de autonoom met encoders goed uit te voeren. (de 3 lengtes zij in milimeters)
    FREIGHTFRENZY_ROBOT(100, 370, 268, 1, 537.6, 312.5);

    private double wheelDiameter;
    private double wheelSeperationWidth;
    private double wheelSeperationLength;
    private double gearRatio;
    private double countsPerRev;
    private double wheelMaxRpm;

    hardwareVariable(double wheelDiameter, double wheelSeperationWidth, double wheelSeperationLength, double gearReatio, double countsPerRev, double wheelMaxRpm) {
        this.wheelDiameter = wheelDiameter;
        this.wheelSeperationLength = wheelSeperationLength;
        this.wheelSeperationWidth = wheelSeperationWidth;
        this.gearRatio = gearReatio;
        this.countsPerRev = countsPerRev;
        this.wheelMaxRpm = wheelMaxRpm;
    }

    public double getWheelDiameter() {
        return wheelDiameter;
    }

    public double getWheelSeperationWidth() {
        return wheelSeperationWidth;
    }

    public double getWheelSeperationLength() {
        return wheelSeperationLength;
    }

    public double getGearRatio() {
        return gearRatio;
    }

    public double getCountsPerRev() {
        return countsPerRev;
    }

    public double getWheelMaxRpm() {
        return wheelMaxRpm;
    }

}
