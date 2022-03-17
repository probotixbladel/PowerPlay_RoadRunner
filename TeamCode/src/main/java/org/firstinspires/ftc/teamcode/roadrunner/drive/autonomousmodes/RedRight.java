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


@Autonomous(name="RedRight", group="probotix")

public class RedRight extends LinearOpMode {

    private hardware Hardware;
    private int HubPos = -400;
    //HubPos= -450 of-1000 of -1510 (laag, midden, hoog)
    OpenCvCamera webcam;
    DeterminationPipeline pipeline = new DeterminationPipeline();


    @Override
    public void runOpMode() throws InterruptedException {
        this.Hardware = new hardware(hardwareMap);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Hardware.getDeliverServo().setPosition(0.35);

        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        //create trajectories
        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .lineToLinearHeading(new Pose2d(25, 25, Math.toRadians(0)))
                .addDisplacementMarker(5,()-> {
                    Hardware.getLiftMotor().setTargetPosition(HubPos);
                    Hardware.getLiftMotor().setPower(0.5);
                })
                .build();

        TrajectorySequence trajSeq2 = drive.trajectorySequenceBuilder(trajSeq.end())
                .lineToLinearHeading(new Pose2d(-2, 0, Math.toRadians(90)))
                .addDisplacementMarker(20,()-> {
                    Hardware.getDeliverServo().setPosition(0.61);
                    Hardware.getLiftMotor().setTargetPosition(0);
                    Hardware.getLiftMotor().setPower(0.5);
                })
                .build();

        TrajectorySequence trajSeq3 = drive.trajectorySequenceBuilder(trajSeq2.end())
                .lineToLinearHeading(new Pose2d(-2, -30, Math.toRadians(90)))

                .build();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.startCameraStream(webcam, 10);
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        while (!opModeIsActive()) {
            telemetry.addData("location", pipeline.getLocation());
            telemetry.addLine("Init Ready!");
            telemetry.update();
            dashboardTelemetry.addData("location", pipeline.getLocation());
            dashboardTelemetry.update();
            if (pipeline.getAnalysis() == LEFT) {
                HubPos = -400;
            } else if (pipeline.getAnalysis() == CENTER) {
                HubPos = -1000;
            } else if (pipeline.getAnalysis() == RIGHT) {
                HubPos = -1700;
            }
        }



        waitForStart();

        if (!isStopRequested()) {
            drive.followTrajectorySequence(trajSeq);
            Hardware.getDeliverServo().setPosition(0);
            sleep(1000);
            drive.followTrajectorySequence(trajSeq2);
            drive.followTrajectorySequence(trajSeq3);

        }
    }
}

