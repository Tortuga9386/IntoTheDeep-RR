package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Auto.ExampleAuto;
import org.firstinspires.ftc.teamcode.opmodes.RobotBase;

public class Lift {
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected RobotBase robotBase;
    public LiftSlide liftSlide;
    public IntakeSlide intakeSlide;
    public IntakeClaw intakeClaw;
    public IntakeLinkage intakelinkage;

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
        intakeSlide = new IntakeSlide();
        intakeClaw = new IntakeClaw();
        intakelinkage = new IntakeLinkage();
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
                targetPosition = 4000;
                goToTarget(targetPosition, slidePower);//highbasket
            } else if (gamepad2.a) {
                targetPosition = 0;
                goToTarget(targetPosition, slidePower);//home
            } else if (gamepad2.x) {
                targetPosition = 1390;
                goToTarget(targetPosition, slidePower);//highchamber
            } else if (gamepad2.b || gamepad2.dpad_down) {
                targetPosition = 300;
                goToTarget(targetPosition, slidePower);//floor pick up position
            } else if (gamepad2.dpad_left) {
                targetPosition = 725;
                goToTarget(targetPosition, slidePower);//score specimen
            } else {
                if (gamepad2.right_stick_y > .1) {
                    targetPosition = targetPosition - 20;
                    goToTarget(targetPosition, slidePower);
                } else if (gamepad2.right_stick_y < -.1) {
                    targetPosition = targetPosition + 20;
                    goToTarget(targetPosition, slidePower);
                }

            }

        }

        public void goToTarget(int targetPosition, double motorPower) {
            //telemetry.addData("goToTarget.targetposition", targetPosition);
            //telemetry.addData("goToTarget.motorPower", motorPower);
            double slideCurrentPosition = liftMotor.getCurrentPosition();
            //if (slideCurrentPosition > liftHomeHeight && slideCurrentPosition < liftMaxHeight) {
            if (true) {
                liftMotor.setTargetPosition(targetPosition);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor.setPower(motorPower);
            }
        }

    }

    public class IntakeSlide {

        public DcMotor intakeliftMotor;

        public IntakeSlide() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        public int liftHomeHeight = 15;
        public int liftMaxHeight = 5000;
        public double intakeslidePowerMan  = 1;
        private int slidetargetPosition = 0;
        public int slidePosition = liftHomeHeight;
        public double intakeslidePowerAuto = 0.5;

        protected void initHardware() {
            intakeliftMotor = hardwareMap.get(DcMotor.class, "bridge");
            intakeliftMotor.setDirection(DcMotor.Direction.REVERSE);
            intakeliftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeliftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }

        public void doIntakeSlideStuff(Gamepad gamepad2) {
            //telemetry.addData("gamepadrightsticky:", gamepad2.right_stick_y);
            //slidePosition = intakeliftMotor.getCurrentPosition();

            //Go forward
                 if (gamepad2.dpad_down) {
                     slidetargetPosition = 1200;
                     goToTarget(slidetargetPosition, intakeslidePowerAuto);//down
                 } else if (gamepad2.dpad_up) {
                     slidetargetPosition = -20;
                     goToTarget(slidetargetPosition, intakeslidePowerMan);//up
            } else {
//                if (gamepad2.left_stick_y > .1) {
//                    slidetargetPosition = slidetargetPosition - 25;
//                    goToTarget(slidetargetPosition, intakeslidePowerMan);
//                } else if (gamepad2.left_stick_y < -.1) {
//                    slidetargetPosition = slidetargetPosition + 25;
//                    goToTarget(slidetargetPosition, intakeslidePowerMan);
//                }

            }

        }

        public void goToTarget(int targetPosition, double motorPower) {
            //telemetry.addData("goToTarget.targetpositionintake", targetPosition);
            //telemetry.addData("goToTarget.motorPowerintake", motorPower);
            double slideCurrentPosition = intakeliftMotor.getCurrentPosition();
            //if (slideCurrentPosition > liftHomeHeight && slideCurrentPosition < liftMaxHeight) {
            if (true) {
                intakeliftMotor.setTargetPosition(targetPosition);
                intakeliftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intakeliftMotor.setPower(motorPower);
            }
        }

    }

    public class IntakeClaw {

        public Servo intakeClaw;

        public IntakeClaw() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

//        public int liftHomeHeight = 15;
//        public int liftMaxHeight = 5000;
//        public double intakeslidePower  = 0;
//        public int targetPosition = liftHomeHeight;
//        public int slidePosition = liftHomeHeight;


        protected void initHardware() {
            intakeClaw = hardwareMap.get(Servo.class, "frontclaw");
        }

        public void doIntakeClawStuff(Gamepad gamepad2) {
            //telemetry.addData("gamepadrightsticky:", gamepad2.right_stick_y);
            //slidePosition = intakeliftMotor.getCurrentPosition();
            intakeClaw.setPosition(0.45+gamepad2.right_trigger * 0.15);

            }
        }

    public class IntakeLinkage {

        public Servo intakeLinkage;

        public IntakeLinkage() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

//        public int liftHomeHeight = 15;
//        public int liftMaxHeight = 5000;
//        public double intakeslidePower  = 0;
//        public int targetPosition = liftHomeHeight;
//        public int slidePosition = liftHomeHeight;


        protected void initHardware() {
            intakeLinkage = hardwareMap.get(Servo.class, "linkage");
        }

        public void doIntakeLinkageStuff(Gamepad gamepad2) {
            //telemetry.addData("gamepadrightsticky:", gamepad2.right_stick_y);
            //slidePosition = intakeliftMotor.getCurrentPosition();
            intakeLinkage.setPosition(0.025+(-0.5 * gamepad2.left_stick_y));

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





