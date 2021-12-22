package org.firstinspires.ftc.teamcode.roadrunner.drive.autonomousmodes;

import static org.firstinspires.ftc.teamcode.probotix.main.openCV.DeterminationPipeline.MarkerPosition.CENTER;
import static org.firstinspires.ftc.teamcode.probotix.main.openCV.DeterminationPipeline.MarkerPosition.LEFT;
import static org.firstinspires.ftc.teamcode.probotix.main.openCV.DeterminationPipeline.MarkerPosition.RIGHT;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.probotix.main.openCV.DeterminationPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvWebcam;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.probotix.main.hardware;
import org.firstinspires.ftc.teamcode.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.trajectorysequence.TrajectorySequence;

@Autonomous(name="BlueRight", group="probotix")

public class BlueRight extends LinearOpMode {

    private hardware Hardware;
    private int HubPos=-450;
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
        //Is this a possible method even during field setup?
        if (pipeline.getAnalysis() == LEFT) {
            HubPos = -1510;
        } else if (pipeline.getAnalysis() == CENTER) {
            HubPos = -1000;
        } else if (pipeline.getAnalysis() == RIGHT) {
            HubPos = -400;
        }

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
              .lineToLinearHeading(new Pose2d(5, -30, Math.toRadians(0)))
              .build();

        TrajectorySequence trajSeqPark = drive.trajectorySequenceBuilder(trajSeq.end())
              .lineToLinearHeading(new Pose2d(49,-28,Math.toRadians(0)))
              .addDisplacementMarker(20,()-> {
                   Hardware.getDeliverServo().setPosition(0.75);
                   Hardware.getLiftMotor().setTargetPosition(0);
                   Hardware.getLiftMotor().setPower(0.5);
                   })
              .build();

        TrajectorySequence trajSeqHub = drive.trajectorySequenceBuilder(trajSeq.end())
             .lineToLinearHeading(new Pose2d(30,15,Math.toRadians(45)))
             .addDisplacementMarker(5,()-> {
                  Hardware.getLiftMotor().setTargetPosition(HubPos);
                  Hardware.getLiftMotor().setPower(0.5);
                  })
             .build();

        waitForStart();

        if (!isStopRequested())

            drive.followTrajectorySequence(trajSeq);
            Hardware.getCarouselMotor().setPower(-0.45);
            sleep(2000);
            Hardware.getCarouselMotor().setPower(0);
            drive.followTrajectorySequence(trajSeqHub);
            //Input code to release preloaded freight
            Hardware.getDeliverServo().setPosition(0.10);
            sleep(1000);
            drive.followTrajectorySequence(trajSeqPark);
        }
}

