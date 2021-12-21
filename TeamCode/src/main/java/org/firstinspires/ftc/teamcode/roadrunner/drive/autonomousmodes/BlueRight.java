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
    private int HubPos=-450;
    //HubPos= -450 of-1000 of -1510 (laag, midden, hoog)

        @Override
        public void runOpMode() throws InterruptedException {
            this.Hardware = new hardware(hardwareMap);

            SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
            Hardware.getDeliverServo().setPosition(0.45);

            Pose2d startPose = new Pose2d(0, 0, 0);

            drive.setPoseEstimate(startPose);

            /*TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(5, -30, Math.toRadians(0)))
                    .addDisplacementMarker(30, () -> {
                        Hardware.getCarouselMotor().setPower(-0.45);
                    })
                    .build();
            */
            // This is to test a way to turn the caroussel. Outside of the trajectory

            TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                    .lineToLinearHeading(new Pose2d(5, -30, Math.toRadians(0)))
                    .build();

            TrajectorySequence trajSeqPark = drive.trajectorySequenceBuilder(trajSeq.end())
                    .lineToLinearHeading(new Pose2d(49,-28,Math.toRadians(0)))
                    .addDisplacementMarker(20,()-> {
                        Hardware.getDeliverServo().setPosition(0.75);
                        Hardware.getLiftMotor().setTargetPosition(0);
                        Hardware.getLiftMotor().setPower(0.4);
                    })
                    .build();

            TrajectorySequence trajSeqHub = drive.trajectorySequenceBuilder(trajSeq.end())
                    .lineToLinearHeading(new Pose2d(30,15,Math.toRadians(45)))
                    .addDisplacementMarker(5,()-> {
                        Hardware.getLiftMotor().setTargetPosition(HubPos);
                        Hardware.getLiftMotor().setPower(0.4);
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

