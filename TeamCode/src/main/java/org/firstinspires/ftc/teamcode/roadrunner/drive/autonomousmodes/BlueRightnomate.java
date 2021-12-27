package org.firstinspires.ftc.teamcode.roadrunner.drive.autonomousmodes;

import static org.firstinspires.ftc.teamcode.probotix.main.openCV.DeterminationPipeline.MarkerPosition.CENTER;
import static org.firstinspires.ftc.teamcode.probotix.main.openCV.DeterminationPipeline.MarkerPosition.LEFT;
import static org.firstinspires.ftc.teamcode.probotix.main.openCV.DeterminationPipeline.MarkerPosition.RIGHT;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.probotix.main.hardware;
import org.firstinspires.ftc.teamcode.probotix.main.openCV.DeterminationPipeline;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(name="BlueRightnomate", group="probotix")

public class BlueRightnomate extends LinearOpMode {

    private hardware Hardware;
    private int HubPos=-400;
    //HubPos= -450 of-1000 of -1510 (laag, midden, hoog)
    OpenCvCamera webcam;
    DeterminationPipeline pipeline = new DeterminationPipeline();


    @Override
    public void runOpMode() throws InterruptedException {
        this.Hardware = new hardware(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Hardware.getDeliverServo().setPosition(0.45);

        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        // This code is from webcamexample


        //Is this a possible method even during field setup?


        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
              .lineToLinearHeading(new Pose2d(5, -30, Math.toRadians(0)))
              .build();

        TrajectorySequence trajSeqHub1 = drive.trajectorySequenceBuilder(trajSeq.end())
                .lineToLinearHeading(new Pose2d(40, -30, Math.toRadians(0)))
                .build();



        TrajectorySequence trajSeqHub2 = drive.trajectorySequenceBuilder(trajSeqHub1.end())
             .lineToLinearHeading(new Pose2d(40,6,Math.toRadians(90)))
             .addDisplacementMarker(5,()-> {
                  Hardware.getLiftMotor().setTargetPosition(HubPos);
                  Hardware.getLiftMotor().setPower(0.5);
                  })
             .build();

        TrajectorySequence trajSeqPark1 = drive.trajectorySequenceBuilder(trajSeqHub2.end())
                .lineToLinearHeading(new Pose2d(45,-28,Math.toRadians(0)))
                .addDisplacementMarker(20,()-> {
                    Hardware.getDeliverServo().setPosition(0.75);
                    Hardware.getLiftMotor().setTargetPosition(0);
                    Hardware.getLiftMotor().setPower(0.5);
                })
                .build();

        TrajectorySequence trajSeqPark2 = drive.trajectorySequenceBuilder(trajSeqPark1.end())
                .lineToLinearHeading(new Pose2d(15, -30, Math.toRadians(-90)))
                .build();

        TrajectorySequence trajSeqPark3 = drive.trajectorySequenceBuilder(trajSeqPark2.end())
                .lineToLinearHeading(new Pose2d(15, 40, Math.toRadians(-90)))
                .build();

        TrajectorySequence trajSeqPark4 = drive.trajectorySequenceBuilder(trajSeqPark3.end())
                .lineToLinearHeading(new Pose2d(-4, 40, Math.toRadians(-90)))
                .build();

        TrajectorySequence trajSeqPark5 = drive.trajectorySequenceBuilder(trajSeqPark4.end())
                .lineToLinearHeading(new Pose2d(-4, 70, Math.toRadians(-90)))
                .build();

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

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(webcam, 10);
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        while (!opModeIsActive()){
            telemetry.addData("location", pipeline.getLocation());
            telemetry.update();
             dashboardTelemetry.addData("location", pipeline.getLocation());
            dashboardTelemetry.update();
            if (pipeline.getAnalysis() == LEFT) {
                HubPos = -400;
            } else if (pipeline.getAnalysis() == CENTER) {
                HubPos = -1000;
            } else if (pipeline.getAnalysis() == RIGHT) {
                HubPos = -1510;
            }
        }

        waitForStart();

        if (!isStopRequested())



            drive.followTrajectorySequence(trajSeq);
            Hardware.getCarouselMotor().setPower(-0.45);
            sleep(2000);
            Hardware.getCarouselMotor().setPower(0);
            drive.followTrajectorySequence(trajSeqHub1);
            drive.followTrajectorySequence(trajSeqHub2);
            //Input code to release preloaded freight
            Hardware.getDeliverServo().setPosition(0.10);
            sleep(1000);
            drive.followTrajectorySequence(trajSeqPark1);
            drive.followTrajectorySequence(trajSeqPark2);
            drive.followTrajectorySequence(trajSeqPark3);
            drive.followTrajectorySequence(trajSeqPark4);
            drive.followTrajectorySequence(trajSeqPark5);
        }
}

