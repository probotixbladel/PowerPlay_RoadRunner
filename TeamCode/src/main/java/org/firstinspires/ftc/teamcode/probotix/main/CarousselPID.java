package org.firstinspires.ftc.teamcode.probotix.main;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MOTOR_VELO_PID;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.probotix.main.enums.hardwareVariable;

@Config
@Autonomous(name="CarousselPID", group="Linear Opmode")
public class CarousselPID extends LinearOpMode {

    public static PIDFCoefficients CAROUSSELPID = new PIDFCoefficients(0, 0, 0,
            0);

    private hardware Hardware;

    public static int speed = 0;

    public double velocity;
    public static double finalVelocity;

    @Override
    public void runOpMode() throws InterruptedException {
        this.Hardware = new hardware(hardwareMap);

        Hardware.init();

        Hardware.getCarouselMotor().setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, CAROUSSELPID);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        while(opModeIsActive()){

            Hardware.getCarouselMotor().setVelocity(speed);

            velocity = Hardware.getCarouselMotor().getVelocity();

            dashboardTelemetry.addData("velocity", velocity);



        }

    }
}
