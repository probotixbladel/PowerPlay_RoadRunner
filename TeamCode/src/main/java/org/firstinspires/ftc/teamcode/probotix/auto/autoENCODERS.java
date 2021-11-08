package org.firstinspires.ftc.teamcode.probotix.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.probotix.main.autohelper;
import org.firstinspires.ftc.teamcode.probotix.main.enums.hardwareVariable;
import org.firstinspires.ftc.teamcode.probotix.main.hardware;

@Autonomous(name="autoENCODERS", group="Linear Opmode")

public class autoENCODERS extends LinearOpMode {

    private hardware Hardware;
    private autohelper Autohelper;


    @Override
    public void runOpMode() {

        this.Hardware = new hardware(hardwareMap);
        this.Autohelper = new autohelper(Hardware, hardwareVariable.FREIGHTFRENZY_ROBOT, this);

        Hardware.init();


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        if(opModeIsActive()) {

            Autohelper.addWheelTicks(0, 0, 0, 0);

            Hardware.getWheelLeftFront().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Hardware.getWheelRightFront().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Hardware.getWheelLeftRear().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Hardware.getWheelRightRear().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

            //Drive to shipping hub
            Autohelper.driveAndWait(60,60,0,5000,5100);
            //Use position of Team Element to control the lift (bakje moet 0,86 voor de servo zijn)

            Hardware.getLiftMotor().setTargetPosition(-1510);
            Hardware.getLiftMotor().setPower(0.4);

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
