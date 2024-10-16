package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.ExampleAuto;
import org.firstinspires.ftc.teamcode.opmodes.RobotBase;

public class Lift {
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected RobotBase robotBase;
    public LiftSlide liftSlide;

    public Lift(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;

        initHardware();
    }
    public Lift(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
    }

    protected void initHardware() {
        liftSlide = new LiftSlide();
    }

    public class LiftSlide {

        public DcMotor liftMotor;

        public LiftSlide() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        public int liftHomeHeight = 15;
        public int liftMaxHeight = 5000;
        public double slidePower  = 0;
        public int targetPosition = liftHomeHeight;
        public int slidePosition = liftHomeHeight;


        protected void initHardware() {
            liftMotor = hardwareMap.get(DcMotor.class, "lift");
            liftMotor.setDirection(DcMotor.Direction.FORWARD);
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void doSlideStuff(Gamepad gamepad2) {
            //telemetry.addData("gamepadrightsticky:", gamepad2.right_stick_y);
            slidePosition = liftMotor.getCurrentPosition();

            //Go up
            slidePower = 1.0;
            if (gamepad2.y) {
                goToTarget(3500, slidePower);
            } else if (gamepad2.a) {
                goToTarget(liftHomeHeight, slidePower);
            } else if (gamepad2.x) {
                goToTarget(3000, slidePower);
            } else if (gamepad2.b) {
                goToTarget(1200, slidePower);
            } else {
                if (gamepad2.left_stick_y > .25) {
                    targetPosition = liftMotor.getCurrentPosition()-20;
                    goToTarget(targetPosition, slidePower);
                } else if (gamepad2.left_stick_y < -.25) {
                    targetPosition = liftMotor.getCurrentPosition()+20;
                    goToTarget(targetPosition, slidePower);
                }

            }

        }

        public void goToTarget(int targetPosition, double motorPower) {
            telemetry.addData("goToTarget.targetposition", targetPosition);
            telemetry.addData("goToTarget.motorPower", motorPower);
            double slideCurrentPosition = liftMotor.getCurrentPosition();
            //if (slideCurrentPosition > liftHomeHeight && slideCurrentPosition < liftMaxHeight) {
            if (true) {
                liftMotor.setTargetPosition(targetPosition);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor.setPower(motorPower);
            }
        }

    }

//    public class IntakeSlide {
//
//        public DcMotor intakeSlideMotor;
//
//
//        public IntakeSlide() { //HardwareMap hardwareMap, RobotBase opMode
//            initHardware();
//        }
//
//        public int liftHomeHeight = 15;
//        public int liftMaxHeight = 500;
//        public double slidePower  = 0;
//        public int targetPosition = liftHomeHeight;
//        public int slidePosition = liftHomeHeight;
//
//        protected void initHardware() {
//            intakeSlideMotor = hardwareMap.get(DcMotor.class, "intake");
//            intakeSlideMotor.setDirection(DcMotor.Direction.FORWARD);
//            intakeSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            intakeSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        }
//
//        public void setIntakeSlideStuff(Gamepad gamepad2) {
//            //telemetry.addData("gamepadrightsticky:", gamepad2.right_stick_y);
//            slidePosition = intakeSlideMotor.getCurrentPosition();
//
//            //Go up
//            slidePower = 1.0;
//            if (gamepad2.y) {
//                goToTarget(500, slidePower);
//            } else if (gamepad2.a) {
//                goToTarget(liftHomeHeight, slidePower);
//            } else if (gamepad2.x) {
//                goToTarget(500, slidePower);
//            } else if (gamepad2.b) {
//                goToTarget(500, slidePower);
//            } else {
//                if (gamepad2.left_stick_y > .25) {
//                    targetPosition = intakeSlideMotor.getCurrentPosition()-20;
//                    goToTarget(targetPosition, slidePower);
//                } else if (gamepad2.left_stick_y < -.25) {
//                    targetPosition = intakeSlideMotor.getCurrentPosition()+20;
//                    goToTarget(targetPosition, slidePower);
//                }
//
//            }
//
//        }
//
//        public void goToTarget(int targetPosition, double motorPower) {
//            telemetry.addData("goToTarget.targetposition", targetPosition);
//            telemetry.addData("goToTarget.motorPower", motorPower);
//            double slideCurrentPosition = intakeSlideMotor.getCurrentPosition();
//            //if (slideCurrentPosition > liftHomeHeight && slideCurrentPosition < liftMaxHeight) {
//            if (true) {
//                intakeSlideMotor.setTargetPosition(targetPosition);
//                intakeSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                intakeSlideMotor.setPower(motorPower);
//            }
//        }
//
//    }



}





