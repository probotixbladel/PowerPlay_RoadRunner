package org.firstinspires.ftc.teamcode.probotix.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

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

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        if(opModeIsActive()) {

            Autohelper.addWheelTicks(0, 0, 0, 0);

            Hardware.getWheelLeftFront().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Hardware.getWheelRightFront().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Hardware.getWheelLeftRear().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            Hardware.getWheelRightRear().setMode(DcMotorEx.RunMode.RUN_TO_POSITION);


            Autohelper.driveAndWait(0,0,0,0,0);
        }

    }



}
