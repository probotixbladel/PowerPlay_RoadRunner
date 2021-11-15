package org.firstinspires.ftc.teamcode.probotix.auto;

import static org.firstinspires.ftc.teamcode.probotix.main.openCV.DeterminationPipeline.MarkerPosition.CENTER;
import static org.firstinspires.ftc.teamcode.probotix.main.openCV.DeterminationPipeline.MarkerPosition.LEFT;
import static org.firstinspires.ftc.teamcode.probotix.main.openCV.DeterminationPipeline.MarkerPosition.RIGHT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.probotix.main.autohelper;
import org.firstinspires.ftc.teamcode.probotix.main.enums.hardwareVariable;
import org.firstinspires.ftc.teamcode.probotix.main.hardware;
import org.firstinspires.ftc.teamcode.probotix.main.openCV.DeterminationPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="autoENCODERS", group="Linear Opmode")

public class autoENCODERS extends LinearOpMode {

    private hardware Hardware;
    private autohelper Autohelper;
    OpenCvCamera webcam;
    DeterminationPipeline pipeline = new DeterminationPipeline();

    @Override
    public void runOpMode() {

        this.Hardware = new hardware(hardwareMap);
        this.Autohelper = new autohelper(Hardware, hardwareVariable.FREIGHTFRENZY_ROBOT, this);

        Hardware.init();
 // This code is from webcamexample
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

        //FtcDashboard dashboard = FtcDashboard.getInstance();
        //dashboard.startCameraStream(webcam, 10);
        //Telemetry dashboardTelemetry = dashboard.getTelemetry();



        while (!opModeIsActive()){
            telemetry.addData("Status", "Initialized");
            telemetry.addData("location", pipeline.getLocation());
            telemetry.update();
            //dashboardTelemetry.addData("location", pipeline.getLocation());
            //dashboardTelemetry.update();
        }


//Until here
        waitForStart();

        if(opModeIsActive()) {

            Autohelper.addWheelTicks(0, 0, 0, 0);

            Hardware.getWheelLeftFront().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Hardware.getWheelRightFront().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Hardware.getWheelLeftRear().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Hardware.getWheelRightRear().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            //Drive to shipping hub
            Autohelper.driveAndWait(60, 60, 0, 5000, 5100);
            if (pipeline.getAnalysis()==LEFT) {
                //Use position of Team Element to control the lift (bakje moet 0,86 voor de servo zijn)

                Hardware.getLiftMotor().setTargetPosition(-470);
            }
            else if (pipeline.getAnalysis()==CENTER){
                Hardware.getLiftMotor().setTargetPosition(-1000);
            }
            else if (pipeline.getAnalysis()==RIGHT){
                Hardware.getLiftMotor().setTargetPosition(-1510);
            }
            Hardware.getLiftMotor().setPower(0.4);
            Hardware.getDeliverServo().setPosition(0.16);

            Hardware.getIntakeMotor().setPower(0.5);

            Hardware.getIntakeMotor().setPower(0);

            //drive to the carousel
            Autohelper.driveAndWait(0,0,0,0,0);
            //turn the carouselmotor

            //drive to storage unit
            Autohelper.driveAndWait(0,0,0,0,0);
            //done

        }

    }



}
