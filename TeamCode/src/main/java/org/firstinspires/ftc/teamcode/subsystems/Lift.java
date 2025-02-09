package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

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
    public Climber climber;
    public Roller roller;
    public Tilter tilter;

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
        climber = new Climber();
        roller = new Roller();
        tilter = new Tilter();
    }

    public class LiftSlide {

        public DcMotor liftMotor;

        public LiftSlide() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        public double slidePower = 0;
        public int targetPosition = 0;


        protected void initHardware() {
            liftMotor = hardwareMap.get(DcMotor.class, "lift");
            liftMotor.setDirection(DcMotor.Direction.FORWARD);
            liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void doSlideStuff(Gamepad gamepad2) {
            goToTarget(targetPosition, slidePower);

            //Go up
            slidePower = 1.0;
            if (gamepad2.y) {
                targetPosition = 3520;
            } else if (gamepad2.a || gamepad2.dpad_up) {
                targetPosition = 0;
            } else if (gamepad2.x) {
                targetPosition = 2150;
            } else if (gamepad2.dpad_left) {
                targetPosition = 1600;
            } else if (gamepad2.b) {
                targetPosition = 210;
            } else if (gamepad2.right_bumper) {
                targetPosition = 400;
            } else if (gamepad2.dpad_down) {
                targetPosition = 35;
            }
                if (gamepad2.right_stick_y > .1) {
                    targetPosition = targetPosition - 20;
                } else if (gamepad2.right_stick_y < -.1) {
                    targetPosition = targetPosition + 20;


            }

        }

        public void goToTarget(int targetPosition, double motorPower) {
            if (true) {
                liftMotor.setTargetPosition(targetPosition);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor.setPower(motorPower);
            }
        }

    }

    public class IntakeSlide {

        public DcMotor intakeliftMotor;
        public TouchSensor resetButton;

        public IntakeSlide() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        private int slidetargetPosition = 0;

        protected void initHardware() {
            intakeliftMotor = hardwareMap.get(DcMotor.class, "bridge");
            intakeliftMotor.setDirection(DcMotor.Direction.REVERSE);
            intakeliftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeliftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            resetButton = hardwareMap.get(TouchSensor.class, "mag1");
        }

        public void doIntakeSlideStuff(Gamepad gamepad2) {

            goToTarget(slidetargetPosition, 1);

            if (gamepad2.dpad_down) {
                slidetargetPosition = 1190;
            }
            if (gamepad2.dpad_up) {
                slidetargetPosition = 10;
            }
            if (slidetargetPosition < 12){
                slidetargetPosition = slidetargetPosition -1;
            }
            if (gamepad2.left_stick_button) {
                slidetargetPosition = slidetargetPosition + 2;

            }
            if (gamepad2.right_stick_button) {
                slidetargetPosition = slidetargetPosition - 2;

            }
            if (resetButton.isPressed()) {
                intakeliftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                slidetargetPosition = 0;
                slidetargetPosition = slidetargetPosition + 1;
            }


        }

        public void goToTarget(int targetPosition, double motorPower) {
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

        protected void initHardware() {
            intakeClaw = hardwareMap.get(Servo.class, "frontclaw");
            intakeClaw.setPosition(0.705);
        }

        public void doIntakeClawStuff(Gamepad gamepad2) {
            if (gamepad2.right_bumper) {
                intakeClaw.setPosition(0.44);
            }
            if (gamepad2.left_bumper) {
                intakeClaw.setPosition(0.65);
            }

            }
        }

    public class Roller {

        private CRServo roller;

        public Roller() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        protected void initHardware() {
            roller = hardwareMap.get(CRServo.class, "roller");
            roller.setDirection(CRServo.Direction.FORWARD);
        }

        public void doIntakeRollerStuff(Gamepad gamepad2) {
            roller.setPower(gamepad2.left_trigger - gamepad2.right_trigger);


        }
    }

    public class Tilter {

        private Servo tilter;

        public Tilter() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        protected void initHardware() {
            tilter = hardwareMap.get(Servo.class, "tilter");
        }

        public void doIntakeTilterStuff(Gamepad gamepad2) {
            if (gamepad2.right_trigger > 0.1 || gamepad2.left_trigger > 0.1){
                tilter.setPosition(0.85);
            }
              else {
                  tilter.setPosition(0.5);
            }

        }
    }

    public class IntakeLinkage {

        public Servo intakeLinkage;

        public IntakeLinkage() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        protected void initHardware() {
            intakeLinkage = hardwareMap.get(Servo.class, "linkage");
        }

        public void doIntakeLinkageStuff(Gamepad gamepad2) {
            intakeLinkage.setPosition(0+(-0.55 * gamepad2.left_stick_y));

            }
        }

    public class Climber {

        public DcMotor climberMotor1;

        public Climber() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }



        protected void initHardware() {
            climberMotor1 = hardwareMap.get(DcMotor.class, "climber");



            climberMotor1.setDirection(DcMotor.Direction.FORWARD);
            climberMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            climberMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        }

        private int targetPosition;

        public void doClimberStuff(Gamepad gamepad1) {
            goToTarget(targetPosition);
            if (gamepad1.right_bumper) {
                targetPosition = 26000;
            } else if (gamepad1.left_bumper) {
                targetPosition = 9615;
            } else if (gamepad1.a){
                targetPosition = 0;
            }

        }



        public void goToTarget(int targetPosition) {
            if (true) {

                climber.climberMotor1.setTargetPosition(targetPosition);
                climber.climberMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                climber.climberMotor1.setPower(1);
            }
        }

        }

    }