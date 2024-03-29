package org.firstinspires.ftc.teamcode.probotix.auto;

import static org.firstinspires.ftc.teamcode.probotix.main.openCV.DeterminationPipeline.MarkerPosition.CENTER;
import static org.firstinspires.ftc.teamcode.probotix.main.openCV.DeterminationPipeline.MarkerPosition.LEFT;
import static org.firstinspires.ftc.teamcode.probotix.main.openCV.DeterminationPipeline.MarkerPosition.RIGHT;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.probotix.main.autohelper;
import org.firstinspires.ftc.teamcode.probotix.main.enums.hardwareVariable;
import org.firstinspires.ftc.teamcode.probotix.main.hardware;
import org.firstinspires.ftc.teamcode.probotix.main.openCV.DeterminationPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Disabled
@Config
@Autonomous(name="autoBlueLeft", group="Linear Opmode")

public class autoBlueLeft extends LinearOpMode {

    private hardware Hardware;
    private autohelper Autohelper;

    @Override
    public void runOpMode() {

        this.Hardware = new hardware(hardwareMap);
        this.Autohelper = new autohelper(Hardware, hardwareVariable.FREIGHTFRENZY_ROBOT, this);

        Hardware.init();


        telemetry.addData("Status", "Initialized");



        waitForStart();

        if(opModeIsActive()) {



            Hardware.getWheelLeftFront().setDirection(DcMotorSimple.Direction.REVERSE);
            Hardware.getWheelLeftRear().setDirection(DcMotorSimple.Direction.REVERSE);

            Hardware.getWheelLeftFront().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Hardware.getWheelRightFront().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Hardware.getWheelLeftRear().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            Hardware.getWheelRightRear().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




            Autohelper.addWheelTicks(0, 0, 0, 0);

            Hardware.getWheelLeftFront().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Hardware.getWheelRightFront().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Hardware.getWheelLeftRear().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Hardware.getWheelRightRear().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            Hardware.getDeliverServo().setPosition(0.45);

            //Drive to shipping hub
            Autohelper.driveAndWait(600.0,0.0, 0.0, 1.5, 1.7);
            Autohelper.driveAndWait(0.0,0.0, -45.0, 0.5, 0.7);
            Autohelper.driveAndWait(-600.0,00.0, 0.0, 1.5, 1.7);

        }

    }
}
