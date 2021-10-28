package org.firstinspires.ftc.teamcode.probotix.auto;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.probotix.main.autohelper;
import org.firstinspires.ftc.teamcode.probotix.main.enums.hardwareVariable;
import org.firstinspires.ftc.teamcode.probotix.main.hardware;

import java.util.Locale;

@Autonomous(name = "AutoIMU", group = "LinearOpMode")
public class autoIMU extends LinearOpMode {

    private hardware Hardware;

    private double previousHeading = 0;
    private double integratedHeading = 0;

    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() throws InterruptedException {


        this.Hardware = new hardware(hardwareMap);

        //aamroepen en instellen van de IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.calibrationDataFile = "BNO55IMUCalibration.json";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        //zorgen dat de wielen remmen als ze geen power meer krijgen ipv doorrollen
        Hardware.getWheelLeftFront().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Hardware.getWheelRightFront().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Hardware.getWheelLeftRear().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        Hardware.getWheelRightRear().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //zorgen dat de motoren van de wielen op encoders werken.
        Hardware.getWheelLeftFront().setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Hardware.getWheelRightFront().setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Hardware.getWheelLeftRear().setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        Hardware.getWheelRightRear().setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        //het aanroepen van de classes
        intHeading obj1 = new intHeading();
        telemetry obj2 = new telemetry();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            //het starten van de loops in de classes
            obj1.start();
            obj2.start();

            sleep(0);

            turnAbsolute(0);

            driveStraight(0,0,0);

            driveSide(0,0,0);

            driveSide(0,0,0);

            driveDiagonalRight(0,0,0,0);

            driveDiagonalLeft(0,0,0,0);

        }

    }

    //Hier maak ik 2 classes aan die Thread extenden zodat ik kan multithreaden oftewel meerdere loops tegelijk kunnen runnen
    //die elkaar niet in de weg lopen.
    class telemetry extends Thread {

        //Deze loop zorgt ervoor dat we alle waardes die we op de telefoon willen zien actief kunnen zien D.M.V telemetry.
        public void run() {

            telemetry.addAction(new Runnable() {
                @Override
                public void run() {
                    angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                }
            });

            while(opModeIsActive()) {
                telemetry.addData("int Heading", integratedHeading);
                telemetry.update();
            }

        }
    }

    class intHeading extends Thread {

        //De IMU (gyroscoop in dit geval) die in de REV hubs zitten ingebouwd gaan automatisch van 180 naar -179 in de code
        //dat is super onhandig als je er in code gebruik van wil maken.
        //Dus deze loop zorgt ervoor dat als je voorbij de 180 gaat de code gewoon graden erbij telt en die waarde opslaat in de cucrrentheiding variabele.
        public void run() {

            while(opModeIsActive()) {
                double currentHeading = imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
                double deltaHeading = currentHeading - previousHeading;

                if (deltaHeading < -180) {
                    deltaHeading += 360;
                } else if (deltaHeading >= 180) {
                    deltaHeading -= 360;
                }
                integratedHeading += deltaHeading;
                previousHeading = currentHeading;

            }
        }
    }

    //Hier draait de robot op de standaart manier door de wielen aan beide kanten de andere kant op te laten draaien.
    //Maar ook dat de wielen stoppen als je in een +-0.2 marge van het gewilde eindpunt verwijdert bent.
    //Dit is de variant met een lagere snelheid voor accurater draaien
    public void turnAbsolute(double target) throws InterruptedException{

        double leftSpeed;
        double rightSpeed;

        Hardware.getWheelLeftFront().setVelocity(0);
        Hardware.getWheelRightFront().setVelocity(0);
        Hardware.getWheelLeftRear().setVelocity(0);
        Hardware.getWheelRightRear().setVelocity(0);

        while (integratedHeading < target - 0.2|| integratedHeading > target + 0.2){

            leftSpeed = (integratedHeading - target)*40;
            rightSpeed = -(integratedHeading - target)*40;

            Hardware.getWheelLeftFront().setVelocity(leftSpeed);
            Hardware.getWheelRightFront().setVelocity(rightSpeed);
            Hardware.getWheelLeftRear().setVelocity(leftSpeed);
            Hardware.getWheelRightRear().setVelocity(rightSpeed);

        }

        Hardware.getWheelLeftFront().setVelocity(0);
        Hardware.getWheelRightFront().setVelocity(0);
        Hardware.getWheelLeftRear().setVelocity(0);
        Hardware.getWheelRightRear().setVelocity(0);



    }


    //Deze werkt precies hetzelde als de normale turnAbsolute
    //Hier draait de robot sneller en met een marge van +-0.5 voor grotere hoeken die minder accuraat hoeven te zijn
    public void turnSAbsolute(double target) throws InterruptedException{

        double leftSpeed;
        double rightSpeed;


        while (integratedHeading < target - 0.5 || integratedHeading > target + 0.5){

            leftSpeed = (integratedHeading - target)*60;
            rightSpeed = -(integratedHeading - target)*60;

            Hardware.getWheelLeftFront().setVelocity(leftSpeed);
            Hardware.getWheelRightFront().setVelocity(rightSpeed);
            Hardware.getWheelLeftRear().setVelocity(leftSpeed);
            Hardware.getWheelRightRear().setVelocity(rightSpeed);

        }

        Hardware.getWheelLeftFront().setVelocity(0);
        Hardware.getWheelRightFront().setVelocity(0);
        Hardware.getWheelLeftRear().setVelocity(0);
        Hardware.getWheelRightRear().setVelocity(0);


    }

    //Simpelweg rechtdoor rijden voor de tijd die je ingevoerd hebt
    public void driveStraight(double velocity, double target, double time) throws InterruptedException{

        double leftSpeed;
        double rightSpeed;

        long startTime = System.currentTimeMillis();



        while ((System.currentTimeMillis()-startTime)<time) {


            leftSpeed = velocity + (integratedHeading - target)*25;
            rightSpeed = velocity - (integratedHeading - target)*25;

            Hardware.getWheelLeftFront().setVelocity(leftSpeed);
            Hardware.getWheelRightFront().setVelocity(rightSpeed);
            Hardware.getWheelLeftRear().setVelocity(leftSpeed);
            Hardware.getWheelRightRear().setVelocity(rightSpeed);

        }

        Hardware.getWheelLeftFront().setVelocity(0);
        Hardware.getWheelRightFront().setVelocity(0);
        Hardware.getWheelLeftRear().setVelocity(0);
        Hardware.getWheelRightRear().setVelocity(0);



    }

    //Simpelweg zijwaarts rijden voor de tijd die je ingevoerd hebt
    public void driveSide(double velocity, double target, double time) throws InterruptedException{

        double lfVelocity;
        double lrVelocity;
        double rfVelocity;
        double rrVelocity;

        long startTime = System.currentTimeMillis();



        while ((System.currentTimeMillis()-startTime)<time) {


            lfVelocity = -velocity + (integratedHeading - target)*25;
            lrVelocity = velocity + (integratedHeading - target)*25;
            rfVelocity = velocity - (integratedHeading - target)*25;
            rrVelocity = -velocity - (integratedHeading - target)*25;


            Hardware.getWheelLeftFront().setVelocity(lfVelocity);
            Hardware.getWheelRightFront().setVelocity(rfVelocity);
            Hardware.getWheelLeftRear().setVelocity(lrVelocity);
            Hardware.getWheelRightRear().setVelocity(rrVelocity);

        }

        Hardware.getWheelLeftFront().setVelocity(0);
        Hardware.getWheelRightFront().setVelocity(0);
        Hardware.getWheelLeftRear().setVelocity(0);
        Hardware.getWheelRightRear().setVelocity(0);



    }

    //diagonaal rijden door in principe de manier van naar voren en zijwaarts te rijden te combineren
    //door 2 wielen tegen elkaar in te laten waardoor met mecanum wielen de robot schuin gaat.
    public void driveDiagonalRight(double dvelocity, double fvelocity, double target, double time) throws InterruptedException{

        double lfVelocity;
        double lrVelocity;
        double rfVelocity;
        double rrVelocity;

        long startTime = System.currentTimeMillis();


        while ((System.currentTimeMillis()-startTime)<time) {


            lfVelocity = fvelocity + dvelocity + (integratedHeading - target)*25;
            lrVelocity = fvelocity + (integratedHeading - target)*25;
            rfVelocity = fvelocity - (integratedHeading - target)*25;
            rrVelocity = fvelocity + dvelocity - (integratedHeading - target)*25;


            Hardware.getWheelLeftFront().setVelocity(lfVelocity);
            Hardware.getWheelRightFront().setVelocity(rfVelocity);
            Hardware.getWheelLeftRear().setVelocity(lrVelocity);
            Hardware.getWheelRightRear().setVelocity(rrVelocity);

        }

        Hardware.getWheelLeftFront().setVelocity(0);
        Hardware.getWheelRightFront().setVelocity(0);
        Hardware.getWheelLeftRear().setVelocity(0);
        Hardware.getWheelRightRear().setVelocity(0);



    }

    //Werkt hetzelde als DiagonalRight maar dan de andere 2 wielen van de robot tegen elkaar in te laten draaien
    public void driveDiagonalLeft(double dvelocity, double fvelocity, double target, double time) throws InterruptedException{

        double lfVelocity;
        double lrVelocity;
        double rfVelocity;
        double rrVelocity;

        long startTime = System.currentTimeMillis();



        while ((System.currentTimeMillis()-startTime)<time) {


            lfVelocity = fvelocity + (integratedHeading - target)*25;
            lrVelocity = fvelocity + dvelocity + (integratedHeading - target)*25;
            rfVelocity = fvelocity + dvelocity - (integratedHeading - target)*25;
            rrVelocity = fvelocity - (integratedHeading - target)*25;


            Hardware.getWheelLeftFront().setVelocity(lfVelocity);
            Hardware.getWheelRightFront().setVelocity(rfVelocity);
            Hardware.getWheelLeftRear().setVelocity(lrVelocity);
            Hardware.getWheelRightRear().setVelocity(rrVelocity);

        }

        Hardware.getWheelLeftFront().setVelocity(0);
        Hardware.getWheelRightFront().setVelocity(0);
        Hardware.getWheelLeftRear().setVelocity(0);
        Hardware.getWheelRightRear().setVelocity(0);



    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    //zorgt ervoor dat de IMU graden teruggeeft en niet radialen
    String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }




}
