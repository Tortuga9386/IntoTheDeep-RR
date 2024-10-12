package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.RobotBase;

public class Lift {
    protected HardwareMap hardwareMap;
    protected Telemetry telemetry;
    protected RobotBase robotBase;
    public    Slide slide;

    public Lift(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;

        initHardware();
    }

    protected void initHardware() {
        slide = new Slide();
    }

    public class Slide {

        public DcMotor slideMotorCenter;

        public Slide() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        public int liftHomeHeight = 15;
        public int liftMaxHeight = 5000;
        public int slidePower  = 0;
        public int targetPosition = liftHomeHeight;
        public int slidePosition = liftHomeHeight;


        protected void initHardware() {
            slideMotorCenter = hardwareMap.get(DcMotor.class, "slide");
            slideMotorCenter.setDirection(DcMotor.Direction.FORWARD);
            slideMotorCenter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slideMotorCenter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

//        public class SlideUp implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    goToTarget(3000, slidePower);
//                    initialized = true;
//                }
//
//                double pos = slideMotorCenter.getCurrentPosition();
//                packet.put("liftPos", pos);
//                if (pos < 3000.0) {
//                    return true;
//                } else {
//                    slideMotorCenter.setPower(0);
//                    return false;
//                }
//            }
//        }
//        public Action SlideUp() {
//            return new SlideUp();
//        }
//        public class SlideHome implements Action {
//            private boolean initialized = false;
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    goToTarget(30, slidePower);
//                    initialized = true;
//                }
//
//                double pos = slideMotorCenter.getCurrentPosition();
//                packet.put("liftPos", pos);
//                if (pos > 30) {
//                    return true;
//                } else {
//                    slideMotorCenter.setPower(0);
//                    return false;
//                }
//            }
//        }
//        public Action SlideHome() {
//            return new SlideHome();
//        }

        public void doSlideStuff(Gamepad gamepad2) {
            //telemetry.addData("gamepadrightsticky:", gamepad2.right_stick_y);
            slidePosition = slideMotorCenter.getCurrentPosition();

            //Go up
            slidePower = 1;
            if (gamepad2.y) {
                goToTarget(5000, slidePower);
            } else if (gamepad2.a) {
                goToTarget(liftHomeHeight, slidePower);
            } else if (gamepad2.x) {
                goToTarget(3000, slidePower);
            } else if (gamepad2.b) {
                goToTarget(1200, slidePower);
            } else {
                if (gamepad2.left_stick_y > .25) {
                    targetPosition = slideMotorCenter.getCurrentPosition()-20;
                    goToTarget(targetPosition, slidePower);
                } else if (gamepad2.left_stick_y < -.25) {
                    targetPosition = slideMotorCenter.getCurrentPosition()+20;
                    goToTarget(targetPosition, slidePower);
                }

            }

        }

        public void goToTarget(int targetPosition, int motorPower) {
            telemetry.addData("goToTarget.targetposition", targetPosition);
            telemetry.addData("goToTarget.motorPower", motorPower);
            double slideCurrentPosition = slideMotorCenter.getCurrentPosition();
            //if (slideCurrentPosition > liftHomeHeight && slideCurrentPosition < liftMaxHeight) {
            if (true) {
                slideMotorCenter.setTargetPosition(targetPosition);
                slideMotorCenter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                slideMotorCenter.setPower(motorPower);
            }
        }

    }

}





