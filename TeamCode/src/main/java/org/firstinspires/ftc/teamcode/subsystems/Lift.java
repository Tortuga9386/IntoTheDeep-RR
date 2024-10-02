package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.RobotBase;

public class Lift {
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected RobotBase robotBase;


    public Bridge bridge;
    public Slide slide;
    public Rotator rotator;
    public Claw claw;

    public boolean allowManualInterrupt = true;


    public Lift(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;

        initHardware();
    }

    protected void initHardware() {
        bridge = new Bridge();
        slide = new Slide();
        rotator = new Rotator();
        claw = new Claw();
    }

    public boolean isBusy() {
        return false;
    }

    public void stop() {
        bridge.stop();
    }

    public class Bridge {
        public DcMotor bridgeMotorCenter;
        private int bridgeUp = 1000;
        private int bridgeDown = 1000;
        private int bridgeSpeed = 1;
        private int bridgeReset = 200;

        public Bridge() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        protected void initHardware() {
            bridgeMotorCenter = hardwareMap.get(DcMotor.class, "bridge");
            bridgeMotorCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            bridgeMotorCenter.setDirection(DcMotor.Direction.FORWARD);
            bridgeMotorCenter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public void up() {
            bridgeMotorCenter.setTargetPosition(bridgeUp);
            bridgeMotorCenter.setPower(bridgeSpeed);
            bridgeMotorCenter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public void down() {
            bridgeMotorCenter.setTargetPosition(bridgeDown);
            bridgeMotorCenter.setPower(bridgeSpeed);
            bridgeMotorCenter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        public boolean isBusy() {
            if (bridgeMotorCenter.isBusy()) {
                return true;
            }
            return false;
        }

        public void reset() {
            bridgeMotorCenter.setTargetPosition(bridgeReset);
            bridgeMotorCenter.setPower(bridgeSpeed);
        }

        public void stop() {
            bridgeMotorCenter.setPower(0.0);
        }
    }

    public class Rotator {
        public Servo servo;

        public boolean allowManualInterrupt = true;
        private double servoOpenPosition = 0.09;
        private double servoClosePosition = 0.015;
        private double servoHomePosition  = 0.015;

        public void Claw(HardwareMap hardwareMap, RobotBase opMode) { initHardware(); }

        protected void initHardware() {
            servo = new Servo();
        }

        public boolean isBusy() {
           if (servo.isBusy()) {
                    return true;
                }
            return false;
        }

        public class Servo {
            private com.qualcomm.robotcore.hardware.Servo servo;
            private boolean rollerUp = true;

            public Servo() { //HardwareMap hardwareMap, RobotBase opMode
                initHardware();
            }

            protected void initHardware() {
                servo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "rotator");
                close();
            }

            public void position(double targetPosition) {
                servo.setPosition(targetPosition);
            }

            public void open() {
                position(servoOpenPosition);
            }

            public void close() {
                position(servoClosePosition);
            }

            public void home() {
                position(servoClosePosition);
            }

            public boolean isBusy() {
                return false;
            }

            public void stop() {
                position(servo.getPosition());
            }
        }

    }

    public class Slide {

        public DcMotor slideMotorCenter;

        public Slide() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        protected void initHardware() {
            slideMotorCenter = hardwareMap.get(DcMotor.class, "slide");
            slideMotorCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotorCenter.setDirection(DcMotor.Direction.FORWARD);
            slideMotorCenter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        public boolean isBusy() {
            return false;
        }

    }

    public class Claw {
        public Servo servo;


        public Claw() { initHardware(); }

        protected void initHardware() {
            servo = new Servo();
        }

        public boolean isBusy() {
            if (servo.isBusy()) {
                    return true;
            }
            return false;
        }

        public void stop() {
            servo.stop();
        }

        public void close() {
            allowManualInterrupt = false;
            servo.close();
        }

        public void open() {
            allowManualInterrupt = false;
            servo.open();
        }

        public class Servo {
            private com.qualcomm.robotcore.hardware.Servo servo;
            private double servoOpenPosition = 0.09;
            private double servoClosePosition = 0.015;

            public Servo() { //HardwareMap hardwareMap, RobotBase opMode
                initHardware();
            }

            protected void initHardware() {
                servo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "frontClaw");
                close();
            }

            public void position(double targetPosition) {
                servo.setPosition(targetPosition);
            }

            public void open() {
                position(servoOpenPosition);
            }

            public void close() {
                position(servoClosePosition);
            }


            public boolean isBusy() {
                return false;
            }

            public void stop() {
                position(servo.getPosition());
            }
        }

    }
}





