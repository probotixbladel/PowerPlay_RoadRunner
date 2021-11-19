package org.firstinspires.ftc.teamcode.probotix.main;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.probotix.main.enums.hardwareVariable;

public class autohelper {

    private hardware Hardware;
    private hardwareVariable HardwareVariable;
    private LinearOpMode opMode;

    //this is to get an instance of the AutoHelper. It needs some information on our robot.
    //IT CAN ONLY BE USED IN A LINEAR OPMODE
    public autohelper(hardware Hardware, hardwareVariable HardwareVariable, LinearOpMode opMode) {
        this.Hardware = Hardware;
        this.HardwareVariable = HardwareVariable;
        this.opMode = opMode;
    }

    //this method drives encoded and waits for it to finish, or reach it's timeout time (IN SECONDS)
    //The parameters are the distance it needs to drive in X Y and Z, the time it needs to take (faster slower etc IN SECONDS) and the time before the code continues anyways
    //linearX is distance forward/backward IN MILLIMETERS
    //linearY is sideways IN MILLIMETERS
    //angularZ is the rotation

    //Je maakt hier een public void aan die je kan aanroepen in een autonome periode zodat je deze berekening niet 100 keer opnieuw
    //in hoeft te voeren maar alleen deze handige class nodig hebt en je de code zo clean mogelijk houdt.
    public void driveAndWait(double linearXmm, double linearYmm, double angularZDEG, double timeS, double timeoutS) {
        driveEncoded(linearXmm, linearYmm, angularZDEG, timeS);
        ElapsedTime runtime = new ElapsedTime();
        runtime.reset();

        //zorgt ervoor dat de robot pas doorgaat met de code als het vorige stukje is afgewerkt oftwel de robot stil staat
        while (opMode.opModeIsActive() && runtime.milliseconds() < timeoutS * 1000 && Hardware.getWheelLeftFront().isBusy() &&
                Hardware.getWheelRightFront().isBusy() && Hardware.getWheelLeftRear().isBusy() &&
                Hardware.getWheelRightRear().isBusy()) {
        }

        Hardware.getWheelLeftFront().setPower(0);
        Hardware.getWheelRightFront().setPower(0);
        Hardware.getWheelLeftRear().setPower(0);
        Hardware.getWheelRightRear().setPower(0);
    }

    //this method drives encoded but doesn't wait for it to finish. The parameters are the distance it needs to drive in X Y and Z and the time it needs to take (faster, slower etc)
    public void driveEncoded(double linearX, double linearY, double angularZ, double time) {

        //De hoek in graden omzetten naar radialen
        double avwB = angularZ / 180 * Math.PI / time;


        //De volgende 3 blokken gebruiken de waardes van de robot zelf om de autonoom op de goede afstanden te laten lopen
        double avwFL = (1 / (HardwareVariable.getWheelDiameter() / 2)) * (linearX / time - linearY / time - (HardwareVariable.getWheelSeperationWidth() + HardwareVariable.getWheelSeperationLength()) / 2 * avwB);
        double avwFR = (1 / (HardwareVariable.getWheelDiameter() / 2)) * (linearX / time + linearY / time + (HardwareVariable.getWheelSeperationWidth() + HardwareVariable.getWheelSeperationLength()) / 2 * avwB);
        double avwRL = (1 / (HardwareVariable.getWheelDiameter() / 2)) * (linearX / time + linearY / time - (HardwareVariable.getWheelSeperationWidth() + HardwareVariable.getWheelSeperationLength()) / 2 * avwB);
        double avwRR = (1 / (HardwareVariable.getWheelDiameter() / 2)) * (linearX / time - linearY / time + (HardwareVariable.getWheelSeperationWidth() + HardwareVariable.getWheelSeperationLength()) / 2 * avwB);

        double rpmFL = (avwFL * 30 / Math.PI) / HardwareVariable.getGearRatio();
        double rpmFR = (avwFR * 30 / Math.PI) / HardwareVariable.getGearRatio();
        double rpmRL = (avwRL * 30 / Math.PI) / HardwareVariable.getGearRatio();
        double rpmRR = (avwRR * 30 / Math.PI) / HardwareVariable.getGearRatio();

        Double ticksFLD = (rpmFL / 60 * HardwareVariable.getCountsPerRev() * time);
        Double ticksFRD = (rpmFR / 60 * HardwareVariable.getCountsPerRev() * time);
        Double ticksRLD = (rpmRL / 60 * HardwareVariable.getCountsPerRev() * time);
        Double ticksRRD = (rpmRR / 60 * HardwareVariable.getCountsPerRev() * time);

        //Zet de waardes van double om in int want doubles hebben te veel getallen achter de comma en zo precies kunnen fysieke robots niet zijn
        int ticksFL = ticksFLD.intValue();
        int ticksFR = ticksFRD.intValue();
        int ticksRL = ticksRLD.intValue();
        int ticksRR = ticksRRD.intValue();

        //Geeft elk wiel het aantal "ticks" oftwel hoe vaak het wiel moet draaien wat in die void gedaan wordt
        addWheelTicks(ticksFL, ticksFR, ticksRL, ticksRR);

        //Zet de snelheid van de wielen aan de hand van hoe snel de wielen kunnen draaien en hoe snel je aangegeven hebt oftwel. Je kunt niet over de max snelheid heen.
        double maxRpm = HardwareVariable.getWheelMaxRpm();
        Hardware.getWheelLeftFront().setPower(rpmFL / maxRpm);
        Hardware.getWheelRightFront().setPower(rpmFR / maxRpm);
        Hardware.getWheelLeftRear().setPower(rpmRL / maxRpm);
        Hardware.getWheelRightRear().setPower(rpmRR / maxRpm);
        //public double Pwr = rpmFL/maxRpm;
    }

    //Voegt het aantal ticks aan het huidige aantal ticks toe zodat de wielen door blijven draaien. Want de wielen onthouden hoever de gedraaid hebben.
    //En nadat dit allemaal berekend is en uitgevoerd beweegt de robot N.A.V. je het hebt ingevoerd (hoop ik :p)
    public void addWheelTicks(int lf, int rf, int lr, int rr) {
        Hardware.getWheelLeftFront().setTargetPosition(lf + Hardware.getWheelLeftFront().getCurrentPosition());
        Hardware.getWheelRightFront().setTargetPosition(rf + Hardware.getWheelRightFront().getCurrentPosition());
        Hardware.getWheelLeftRear().setTargetPosition(lr + Hardware.getWheelLeftRear().getCurrentPosition());
        Hardware.getWheelRightRear().setTargetPosition(rr + Hardware.getWheelRightRear().getCurrentPosition());
    }

}
