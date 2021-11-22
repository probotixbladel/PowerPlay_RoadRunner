package org.firstinspires.ftc.teamcode.probotix.drive;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.probotix.main.hardware;

@Disabled
@TeleOp(name="basicdrive", group="Drive")
public class basicdrive extends OpMode {

    private hardware Hardware;

    @Override
    public void init() {
        Hardware = new hardware(hardwareMap);
        Hardware.init();



        telemetry.addData("Status", "Initialized");
        telemetry.update();

    }

    @Override
    public void loop() {

        if (Hardware.getGear() == null) {
            Hardware.setGear(hardware.Gear.SECOND);
        }

        if (gamepad1.a) {
            Hardware.setGear(hardware.Gear.FIRST);
        } else if (gamepad1.b) {
            Hardware.setGear(hardware.Gear.SECOND);
        } else if (gamepad1.x) {
            Hardware.setGear(hardware.Gear.THIRD);
        } else if (gamepad1.y) {
            Hardware.setGear(hardware.Gear.FOURTH);
        }

        double speed = gamepad1.right_trigger * Hardware.getGear().getMaxSpeed();
        double fl = 0;
        double fr = 0;
        double rl = 0;
        double rr = 0;
        boolean left = gamepad1.dpad_left;
        boolean right = gamepad1.dpad_right;
        boolean up = gamepad1.dpad_up;
        boolean down = gamepad1.dpad_down;
        if (left) {
            fl = -speed;
            fr = speed;
            rl = speed;
            rr = -speed;
        }
        if (down) {
            fl = -speed;
            fr = -speed;
            rl = -speed;
            rr = -speed;
        }
        if (right) {
            fl = speed;
            fr = -speed;
            rl = -speed;
            rr = speed;
        }
        if (up) {
            fl = speed;
            fr = speed;
            rl = speed;
            rr = speed;
        }
        if (left && up) {
            fl = 0;
            fr = speed;
            rl = speed;
            rr = 0;
        }
        if (left && down) {
            fl = -speed;
            fr = 0;
            rl = 0;
            rr = -speed;
        }
        if (right && up) {
            fl = speed;
            fr = 0;
            rl = 0;
            rr = speed;
        }
        if (right && down) {
            fl = 0;
            fr = -speed;
            rl = -speed;
            rr = 0;
        }
        double rotation = gamepad1.right_stick_x;
        fl += rotation*0.47;
        fr -= rotation*0.47;
        rl += rotation*0.47;
        rr -= rotation*0.47;

        Hardware.getWheelLeftFront().setPower(fl);
        Hardware.getWheelRightFront().setPower(fr);
        Hardware.getWheelLeftRear().setPower(rl);
        Hardware.getWheelRightRear().setPower(rr);

    }
}
