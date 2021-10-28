package org.firstinspires.ftc.teamcode.probotix.main;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.probotix.drive.basicdrive;


public class hardware {
    private DcMotorEx wheelLeftFront, wheelRightFront, wheelLeftRear, wheelRightRear;


    private HardwareMap hardwareMap;
    private Gear gear;

    public hardware(basicdrive opMode) {
        this.hardwareMap = opMode.hardwareMap;
        this.gear = Gear.FOURTH;
        reset();
    }


    public hardware(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        init();
    }

    public void init() {
        this.wheelLeftFront = (DcMotorEx) hardwareMap.dcMotor.get("wheelLeftFront");
        this.wheelRightFront = (DcMotorEx) hardwareMap.dcMotor.get("wheelRightFront");
        this.wheelLeftRear = (DcMotorEx) hardwareMap.dcMotor.get("wheelLeftRear");
        this.wheelRightRear = (DcMotorEx) hardwareMap.dcMotor.get("wheelRightRear");
        reset();
    }

    public void reset() {
        wheelLeftFront.setPower(0);
        wheelRightFront.setPower(0);
        wheelLeftRear.setPower(0);
        wheelRightRear.setPower(0);

        wheelLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        wheelLeftRear.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setGear(Gear gear) {
        this.gear = gear;
    }

    public Gear getGear() {
        return this.gear;
    }

    public enum Gear {
        FIRST(0.25), SECOND(0.5), THIRD(0.75), FOURTH(1.0);

        private double MaxSpeed;

        Gear(double maxSpeed){
            this.MaxSpeed = maxSpeed;
        }

        public double getMaxSpeed() {
            return MaxSpeed;
        }
    }



    public DcMotorEx getWheelLeftFront() {
        return wheelLeftFront;
    }

    public DcMotorEx getWheelRightFront() {
        return wheelRightFront;
    }

    public DcMotorEx getWheelLeftRear() {
        return wheelLeftRear;
    }

    public DcMotorEx getWheelRightRear() {
        return wheelRightRear;
    }

}
