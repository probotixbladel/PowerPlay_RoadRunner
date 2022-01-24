package org.firstinspires.ftc.teamcode.probotix.main;

import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.MOTOR_VELO_PID;
import static org.firstinspires.ftc.teamcode.roadrunner.drive.DriveConstants.RUN_USING_ENCODER;

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

  /*  public static PIDFCoefficients CAROUSSELPID = new PIDFCoefficients(0, 0, 0,
            0);*/

    public static int P = 0;
    public static int I = 0;
    public static int D = 0;
    public static int F = 0;


    private hardware Hardware;

    public static double speed = 0;

    public static int upperspeed = 4000;
    public static int lowerspeed = 0;

    public double velocity;


    @Override
    public void runOpMode() throws InterruptedException {
        this.Hardware = new hardware(hardwareMap);

        Hardware.init();

        Hardware.getCarouselMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Hardware.getCarouselMotor().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Hardware.getCarouselMotor().setVelocityPIDFCoefficients(P,I,D,F);
       

        //Hardware.getCarouselMotor().setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, CAROUSSELPID);

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

            Hardware.getCarouselMotor().setPower(speed);

            velocity = Hardware.getCarouselMotor().getVelocity();

            dashboardTelemetry.addData("velocity", velocity);
            dashboardTelemetry.addData("speed", speed);
            dashboardTelemetry.addData("upper", upperspeed);
            dashboardTelemetry.addData("lower", lowerspeed);

            telemetry.addData("velocity", velocity);
            telemetry.update();
            dashboardTelemetry.update();



        }

    }
}
