package org.firstinspires.ftc.teamcode.probotix.auto;

import static org.firstinspires.ftc.teamcode.probotix.main.openCV.DeterminationPipeline.MarkerPosition.CENTER;
import static org.firstinspires.ftc.teamcode.probotix.main.openCV.DeterminationPipeline.MarkerPosition.LEFT;
import static org.firstinspires.ftc.teamcode.probotix.main.openCV.DeterminationPipeline.MarkerPosition.RIGHT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.probotix.main.autohelper;
import org.firstinspires.ftc.teamcode.probotix.main.enums.hardwareVariable;
import org.firstinspires.ftc.teamcode.probotix.main.hardware;
import org.firstinspires.ftc.teamcode.probotix.main.openCV.DeterminationPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Config
@Autonomous(name="BlueRightOP", group="Linear Opmode")

public class autoENCODERS extends LinearOpMode {

    private hardware Hardware;
    private autohelper Autohelper;
    //OpenCvCamera webcam;
    //eterminationPipeline pipeline = new DeterminationPipeline();

    public double positionlf;
    public double positionrf;
    public double positionlr;
    public double positionrr;

    @Override
    public void runOpMode() {

        this.Hardware = new hardware(hardwareMap);
        this.Autohelper = new autohelper(Hardware, hardwareVariable.FREIGHTFRENZY_ROBOT, this);

        Hardware.init();
 // This code is from webcamexample

        /*
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"),cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }
            @Override
            public void onError(int errorCode){}
        });

         */




       /* while (!opModeIsActive()){
            telemetry.addData("Status", "Initialized");
            //telemetry.addData("location", pipeline.getLocation());
            //telemetry.addData("test", "test");
            telemetry.update();
            //dashboardTelemetry.addData("location", pipeline.getLocation());
            //dashboardTelemetry.update();
        }

        */

        telemetry.addData("Status", "Initialized");


//Until here
        waitForStart();

        if(opModeIsActive()) {


            //telemetry.addData("idk", "idk");
            //telemetry.update();
            Hardware.getWheelLeftFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Hardware.getWheelLeftRear().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Hardware.getWheelRightFront().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Hardware.getWheelRightRear().setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            Hardware.getWheelLeftFront().setDirection(DcMotorSimple.Direction.REVERSE);
            Hardware.getWheelLeftRear().setDirection(DcMotorSimple.Direction.REVERSE);

            Hardware.getWheelLeftFront().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Hardware.getWheelRightFront().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Hardware.getWheelLeftRear().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Hardware.getWheelRightRear().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



            Hardware.getWheelLeftFront().setTargetPosition(0);
            Hardware.getWheelRightFront().setTargetPosition(0);
            Hardware.getWheelLeftRear().setTargetPosition(0);
            Hardware.getWheelRightRear().setTargetPosition(0);

            Autohelper.addWheelTicks(0, 0, 0, 0);

            Hardware.getWheelLeftFront().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Hardware.getWheelRightFront().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Hardware.getWheelLeftRear().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Hardware.getWheelRightRear().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            Hardware.getDeliverServo().setPosition(0.45);
            //Hardware.getLiftMotor().setTargetPosition(0);

            //Drive to shipping hub
            Autohelper.driveAndWait(0, -600, 0, 3, 3.5);
            Autohelper.driveAndWait(0,-50,0,0.5,1);

            Hardware.getCarouselMotor().setPower(-0.45);

            sleep(2300);

            Autohelper.driveAndWait(0,100,0,1,1.5);
            Hardware.getCarouselMotor().setPower(0);
            Autohelper.driveAndWait(-100,0,0,1,1.5);
            Autohelper.driveAndWait(200,0,0,2,2.5);


            /*
            //Autohelper.driveAndWait(100,0,0,1,1.5);
            Autohelper.driveAndWait(0,1250,0,9,9.5);



            sleep(100);

            if (pipeline.getAnalysis() == LEFT) {
                Hardware.getLiftMotor().setTargetPosition(-1510);
                Hardware.getLiftMotor().setPower(0.6);
            } else if (pipeline.getAnalysis() == CENTER) {
                Hardware.getLiftMotor().setTargetPosition(-1000);
                Hardware.getLiftMotor().setPower(0.6);
            } else if (pipeline.getAnalysis() == RIGHT) {
                Hardware.getLiftMotor().setTargetPosition(-400);
                Hardware.getLiftMotor().setPower(0.6);
            }

            Autohelper.driveAndWait(250,0,0,2,2.5);

            sleep(1000);

            Hardware.getDeliverServo().setPosition(0.12);

            sleep(4000);

            Hardware.getLiftMotor().setTargetPosition(0);
            Hardware.getLiftMotor().setPower(0.4);
            Hardware.getDeliverServo().setPosition(0.75);

            sleep(3000);

             */






        }

    }
}
