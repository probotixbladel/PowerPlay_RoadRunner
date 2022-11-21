package org.firstinspires.ftc.teamcode.probotix.main;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;


public class hardware {
        public DcMotorEx wheelLeftFront, wheelRightFront, wheelLeftRear, wheelRightRear, liftMotor;
        private Servo grabServo;

        private HardwareMap hardwareMap;
        private Gear gear;



        public hardware(HardwareMap hardwareMap) {
            this.hardwareMap = hardwareMap;
            init();
        }

        public void init() {
            this.wheelLeftFront = (DcMotorEx) hardwareMap.dcMotor.get("wheelLeftFront");
            this.wheelRightFront = (DcMotorEx) hardwareMap.dcMotor.get("wheelRightFront");
            this.wheelLeftRear = (DcMotorEx) hardwareMap.dcMotor.get("wheelLeftRear");
            this.wheelRightRear = (DcMotorEx) hardwareMap.dcMotor.get("wheelRightRear");
            this.liftMotor = (DcMotorEx) hardwareMap.dcMotor.get("liftMotor");
            this.grabServo = (Servo) hardwareMap.servo.get("grabServo");
            reset();
        }

        public void reset() {
            wheelLeftFront.setPower(0);
            wheelRightFront.setPower(0);
            wheelLeftRear.setPower(0);
            wheelRightRear.setPower(0);

            liftMotor.setPower(0);
            liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setTargetPosition(0);
            liftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

            grabServo.setPosition(0.8);



            //deliverServo.setPosition(0.75);

            // wheelLeftFront.setDirection(DcMotorSimple.Direction.REVERSE);
            // wheelLeftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        }

        public void setGear(Gear gear) {
            this.gear = gear;
        }

        public Gear getGear() {
            return this.gear;
        }

        public enum  Gear {
            FIRST(0.25), SECOND(0.5), THIRD(0.75), FOURTH(0.90);
            //FOURTH was 1.0

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

        public DcMotorEx getLiftMotor() {
        return liftMotor;
    }

        public Servo getGrabServo(){return grabServo;}


    }

