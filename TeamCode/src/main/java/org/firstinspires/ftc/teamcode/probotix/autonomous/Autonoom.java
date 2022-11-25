package org.firstinspires.ftc.teamcode.probotix.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.probotix.main.AdvancedPipeline;
import org.firstinspires.ftc.teamcode.probotix.main.hardware;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name="Autonoom", group="probotix")


public class Autonoom extends LinearOpMode {
    private hardware Hardware;
    private AdvancedPipeline pipeline;
    OpenCvCamera webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        this.Hardware = new hardware(hardwareMap);


        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // We want to start the bot at x: 0, y: 0, heading: 0 degrees
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        drive.setPoseEstimate(startPose);

        Trajectory traj1 = drive.trajectoryBuilder(startPose)
                .splineTo(new Vector2d(5, 0), Math.toRadians(0))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(-5, 0), Math.toRadians(0))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(0, 5), Math.toRadians(0))
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


        waitForStart();

        if (!isStopRequested())
            if (pipeline.getColour() == AdvancedPipeline.SignalColour.BLUE){
                drive.followTrajectory(traj1);
            }
        else if (pipeline.getColour() == AdvancedPipeline.SignalColour.RED){
                drive.followTrajectory(traj2);
            }
        else {
            drive.followTrajectory(traj3);
            }
            //drive.followTrajectory(traj1);
        //drive.followTrajectory(traj2);
    }
}

