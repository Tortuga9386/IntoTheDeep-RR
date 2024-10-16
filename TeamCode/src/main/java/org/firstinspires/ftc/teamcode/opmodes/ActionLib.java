package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ActionLib {

    public ActionLib() {

    }

    public static class RobotLift {
        private DcMotor lift;

        public RobotLift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotor.class, "lift");
            lift.setDirection(DcMotor.Direction.FORWARD);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {
                    lift.setPower(0.5);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }
        public class LiftSpecimen implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {
                    lift.setPower(0.5);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 1200.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftSpecimen() {
            return new LiftSpecimen();
        }
        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.3);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 50.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }

    public static class RobotIntakeSlide {
        private DcMotor intakeSlide;

        public RobotIntakeSlide(HardwareMap hardwareMap) {
            intakeSlide = hardwareMap.get(DcMotor.class, "slides");
            intakeSlide.setDirection(DcMotor.Direction.FORWARD);
            intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public class LiftUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {
                    intakeSlide.setPower(0.5);
                    initialized = true;
                }

                double pos = intakeSlide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 3000.0) {
                    return true;
                } else {
                    intakeSlide.setPower(0);
                    return false;
                }
            }
        }
        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeSlide.setPower(-0.5);
                    initialized = true;
                }

                double pos = intakeSlide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 50.0) {
                    return true;
                } else {
                    intakeSlide.setPower(0);
                    return false;
                }
            }
        }
        public Action liftDown(){
            return new LiftDown();
        }
    }

    public static class RobotIntakeRotator {
        private Servo intakeRotatorServo;
        private double rightPosition = 0.7;
        private double leftPosition = 0.3;

        public RobotIntakeRotator(HardwareMap hardwareMap) {
            intakeRotatorServo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "rotatorServo");
            intakeRotatorServo.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
        }

        public class RotateLeft implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {
                    intakeRotatorServo.setPosition(leftPosition);
                    initialized = true;
                }

                double pos = intakeRotatorServo.getPosition();
                packet.put("clawPos", pos);
                if (pos >= leftPosition) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        public Action rotateLeft() {
            return new RotateLeft();
        }

        public class RotateRight implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeRotatorServo.setPosition(rightPosition);
                    initialized = true;
                }

                double pos = intakeRotatorServo.getPosition();
                packet.put("clawPos", pos);
                if (pos <= rightPosition) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        public Action rotateRight(){
            return new RotateRight();
        }

    }

    public static class RobotIntakeClaw {
        private Servo intakeClawServo;
        private double openPosition = 0.7;
        private double closedPosition = 0.3;

        public RobotIntakeClaw(HardwareMap hardwareMap) {
            intakeClawServo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "clawServo");
            intakeClawServo.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
        }

        public class ClawOpen implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {

                if (!initialized) {
                    intakeClawServo.setPosition(openPosition);
                    initialized = true;
                }

                double pos = intakeClawServo.getPosition();
                packet.put("clawPos", pos);
                if (pos >= openPosition) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        public Action clawOpen() {
            return new ClawOpen();
        }

        public class ClawClosed implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeClawServo.setPosition(closedPosition);
                    initialized = true;
                }

                double pos = intakeClawServo.getPosition();
                packet.put("clawPos", pos);
                if (pos <= closedPosition) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        public Action clawClosed(){
            return new ClawClosed();
        }

    }

}





