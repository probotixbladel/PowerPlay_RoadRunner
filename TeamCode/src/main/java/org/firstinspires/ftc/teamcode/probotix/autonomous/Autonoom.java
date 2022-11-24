package org.firstinspires.ftc.teamcode.probotix.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.probotix.main.hardware;

@Autonomous(name="Autonoom", group="probotix")


public class Autonoom extends LinearOpMode {
    private hardware Hardware;

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
                .splineTo(new Vector2d(10, 0), Math.toRadians(0))
                .build();


        waitForStart();

        if (!isStopRequested())

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
    }
}

