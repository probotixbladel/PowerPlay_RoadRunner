package org.firstinspires.ftc.teamcode.roadrunner.drive.autonomousmodes;

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
        @Override
        public void runOpMode() throws InterruptedException {
            this.Hardware = new hardware(hardwareMap);

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

            Pose2d startPose = new Pose2d(0, 0, 0);

            drive.setPoseEstimate(startPose);

            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(5, -30, Math.toRadians(0)))
                    .addDisplacementMarker(30, () -> {
                        Hardware.getCarouselMotor().setPower(-0.45);
                    })



                    .build();

            waitForStart();

            if (!isStopRequested())
                drive.followTrajectorySequence(trajSeq);
        }
}

