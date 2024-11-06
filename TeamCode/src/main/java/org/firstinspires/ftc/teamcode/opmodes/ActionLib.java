package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ActionLib {

    public ActionLib() {

    }

    static int motorPrecision = 50;
    static double servoPrecision = 0.001;

//////////////////////////////////////////////////////////////////////////
///  ROBOT LIFT
//////////////////////////////////////////////////////////////////////////

    public static class RobotLift {
        private DcMotor lift;
        private boolean quitter = true;

        public RobotLift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotor.class, "lift");
            lift.setDirection(DcMotor.Direction.FORWARD);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void disableLiftHold() {
            quitter = true;
        }

        public void goToTarget(int targetPosition, double motorPower) {
            double slideCurrentPosition = lift.getCurrentPosition();
            if (true) {
                lift.setTargetPosition(targetPosition);
                lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                lift.setPower(motorPower);
            }
        }

        public class ActionLiftUp implements Action {
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
        public Action actionLiftUp() { return new ActionLiftUp(); }

        public class ActionLiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                int targetPosition = 0;

                double currentPosition = lift.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if ((Math.abs(targetPosition - currentPosition) > motorPrecision) || !quitter) {
                    goToTarget(targetPosition,0.5);
                    return true;
                } else {
                    if (quitter) {
                        quitter = true;
                        return false;
                    }
                    return true;
                }
            }
        }
        public Action actionLiftDown(){
            return new ActionLiftDown();
        }

        public class ActionLiftSpecimen implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Log.v("ActionLiftSpecimen", "START/////////////////////////////////////////////////////////////////");
                int targetPosition = 1390;

                double currentPosition = lift.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if ((Math.abs(targetPosition - currentPosition) > motorPrecision) || !quitter) {
                    Log.v("ActionLiftSpecimen", "RUNNING//////targetPosition"+targetPosition+ " vs currentPosition"+currentPosition+"////////////////////////////////////////");
                    Log.v("ActionLiftSpecimen", "RUNNING//////testVal:"+Math.abs(targetPosition - currentPosition)+" > 50///////////////////////////////////////");
                    goToTarget(targetPosition,0.5);
                    return true;
                } else {
                    if (quitter) {
                        quitter = true;
                        Log.v("ActionLiftSpecimen", "FINISH//////I QUIT!!!!////////////////////////////////////////");
                        Log.v("ActionLiftSpecimen", "FINISH//////testVal:"+Math.abs(targetPosition - currentPosition)+" < 50///////////////////////////////////////");
                        Log.v("ActionLiftSpecimen", "FINISH//////////////////////////////////////////////////////////////////");
                        return false;
                    }
                    Log.v("ActionLiftSpecimen", "FINISH//////ON TARGET BUT NOT A QUITTER!////////////////////////////////////////");
                    Log.v("ActionLiftSpecimen", "FINISH//////testVal:"+Math.abs(targetPosition - currentPosition)+" < 50///////////////////////////////////////");
                    Log.v("ActionLiftSpecimen", "FINISH//////////////////////////////////////////////////////////////////");
                    return true;
                }
            }
        }
        public Action actionLiftSpecimen() {
            return new ActionLiftSpecimen();
        }

        public class ActionLiftScore implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                int targetPosition = 650;

                double currentPosition = lift.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if ((Math.abs(targetPosition - currentPosition) > motorPrecision) || !quitter) {
                    goToTarget(targetPosition,0.5);
                    return true;
                } else {
                    if (quitter) {
                        quitter = true;
                        return false;
                    }
                    return true;
                }
            }
        }
        public Action actionliftScore(){ return new ActionLiftScore(); }

        public class ActionClawGrab implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Log.v("ActionClawGrab", "START/////////////////////////////////////////////////////////////////");
                int targetPosition = 240;

                double currentPosition = lift.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if ((Math.abs(targetPosition - currentPosition) > motorPrecision) || !quitter) {
                    goToTarget(targetPosition,0.7);
                    Log.v("ActionClawGrab", "RUNNING//////targetPosition"+targetPosition+ " vs currentPosition"+currentPosition+"////////////////////////////////////////");
                    Log.v("ActionClawGrab", "RUNNING//////testVal:"+Math.abs(targetPosition - currentPosition)+" > 50///////////////////////////////////////");
                    return true;
                } else {
                    if (quitter) {
                        quitter = true;
                        Log.v("ActionClawGrab", "FINISH//////I QUIT!!!!////////////////////////////////////////");
                        Log.v("ActionClawGrab", "FINISH//////testVal:"+Math.abs(targetPosition - currentPosition)+" < 50///////////////////////////////////////");
                        Log.v("ActionClawGrab", "FINISH//////////////////////////////////////////////////////////////////");
                        return false;
                    } else {
                        Log.v("ActionClawGrab", "FINISH//////ON TARGET BUT NOT A QUITTER!////////////////////////////////////////");
                        Log.v("ActionClawGrab", "FINISH//////testVal:"+Math.abs(targetPosition - currentPosition)+" < 50///////////////////////////////////////");
                        Log.v("ActionClawGrab", "FINISH//////////////////////////////////////////////////////////////////");
                    }
                    return true;
                }
            }
        }
        public Action actionClawGrab(){ return new ActionClawGrab(); }

        public class ActionLiftToBasket implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Log.v("ActionLiftToBasket", "START/////////////////////////////////////////////////////////////////");
                int targetPosition = 4000;

                double currentPosition = lift.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if ((Math.abs(targetPosition - currentPosition) > motorPrecision) || !quitter) {
                    goToTarget(targetPosition,0.7);
                    Log.v("ActionLiftToBasket", "RUNNING//////targetPosition"+targetPosition+ " vs currentPosition"+currentPosition+"////////////////////////////////////////");
                    Log.v("ActionLiftToBasket", "RUNNING//////testVal:"+Math.abs(targetPosition - currentPosition)+" > 50///////////////////////////////////////");
                    return true;
                } else {
                    if (quitter) {
                        quitter = true;
                        Log.v("ActionLiftToBasket", "FINISH//////I QUIT!!!!////////////////////////////////////////");
                        Log.v("ActionLiftToBasket", "FINISH//////testVal:"+Math.abs(targetPosition - currentPosition)+" < 50///////////////////////////////////////");
                        Log.v("ActionLiftToBasket", "FINISH//////////////////////////////////////////////////////////////////");
                        return false;
                    } else {
                        Log.v("ActionLiftToBasket", "FINISH//////ON TARGET BUT NOT A QUITTER!////////////////////////////////////////");
                        Log.v("ActionLiftToBasket", "FINISH//////testVal:"+Math.abs(targetPosition - currentPosition)+" < 50///////////////////////////////////////");
                        Log.v("ActionLiftToBasket", "FINISH//////////////////////////////////////////////////////////////////");
                    }
                    return true;
                }
            }
        }
        public Action actionLiftToBasket(){ return new ActionLiftToBasket(); }

        public class ActionQuitLift implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                disableLiftHold();
                return false;
            }
        }
        public Action actionQuitLift(){ return new ActionQuitLift(); }

        public class ActionHoldLift implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                quitter = false;
                return false;
            }
        }
        public Action actionHoldLift(){ return new ActionHoldLift(); }

    }

//////////////////////////////////////////////////////////////////////////
///  INTAKE SLIDE
//////////////////////////////////////////////////////////////////////////

    public static class RobotIntakeSlide {
        private DcMotor intakeSlide;

        public RobotIntakeSlide(HardwareMap hardwareMap) {
            intakeSlide = hardwareMap.get(DcMotor.class, "bridge");
            intakeSlide.setDirection(DcMotor.Direction.REVERSE);
            intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void goToTarget(int targetPosition, double motorPower) {
            double slideCurrentPosition = intakeSlide.getCurrentPosition();
            if (true) {
                intakeSlide.setTargetPosition(targetPosition);
                intakeSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                intakeSlide.setPower(motorPower);
            }
        }

        public class ActionRetract implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                int targetPosition = 0;

                if (!initialized) {
                    goToTarget(targetPosition,0.5);
                    initialized = true;
                }

                double currentPosition = intakeSlide.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if (Math.abs(targetPosition - currentPosition) > 50) {
                    return true;
                } else {
                    intakeSlide.setPower(0);
                    return false;
                }
            }
        }
        public Action actionRetract() {
            return new ActionRetract();
        }

        public class ActionReach implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                int targetPosition = 1150;

                if (!initialized) {
                    goToTarget(targetPosition,0.5);
                    initialized = true;
                }

                double currentPosition = intakeSlide.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if (Math.abs(targetPosition - currentPosition) > 50) {
                    return true;
                } else {
                    intakeSlide.setPower(0);
                    return false;
                }
            }
        }
        public Action actionReach(){
            return new ActionReach();
        }
    }

//////////////////////////////////////////////////////////////////////////
///  ROBOT INTAKE
//////////////////////////////////////////////////////////////////////////

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

//////////////////////////////////////////////////////////////////////////
///  INTAKE CLAW
//////////////////////////////////////////////////////////////////////////


    public static class RobotIntakeClaw {
        private Servo intakeClawServo;
        private double openPosition = 0.6;
        private double closedPosition = 0.45;

        public RobotIntakeClaw(HardwareMap hardwareMap) {
            intakeClawServo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "frontclaw");
            intakeClawServo.setDirection(Servo.Direction.FORWARD);
        }

        public void clawOpen() {
            intakeClawServo.setPosition(openPosition);
        }
        public void clawClose() {
            intakeClawServo.setPosition(closedPosition);
        }

        public class ActionClawOpen implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Log.v("ActionClawOpen", "START/////////////////////////////////////////////////////////////////");

                if (!initialized) {
                    intakeClawServo.setPosition(openPosition);
                    initialized = true;
                }

                double currentPosition = intakeClawServo.getPosition();
                packet.put("clawPos", currentPosition);
                if (Math.abs(openPosition - currentPosition) > servoPrecision) {
                    intakeClawServo.setPosition(openPosition);
//                    Log.v("ActionClawGrab", "RUNNING//////targetPosition"+openPosition+ " vs currentPosition"+currentPosition+"////////////////////////////////////////");
//                    Log.v("ActionClawGrab", "RUNNING//////testVal:"+Math.abs(openPosition - currentPosition)+" > 50///////////////////////////////////////");
                    return true;
                } else {
                  return false;
                }

            }
        }
        public Action actionClawOpen() {
            return new ActionClawOpen();
        }

        public class ActionClawClose implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeClawServo.setPosition(closedPosition);
                    initialized = true;
                }

                double currentPosition = intakeClawServo.getPosition();
                packet.put("clawPos", currentPosition);
                if (Math.abs(closedPosition - currentPosition) > servoPrecision) {
                    intakeClawServo.setPosition(closedPosition);
//                    Log.v("ActionClawGrab", "RUNNING//////targetPosition"+openPosition+ " vs currentPosition"+currentPosition+"////////////////////////////////////////");
//                    Log.v("ActionClawGrab", "RUNNING//////testVal:"+Math.abs(openPosition - currentPosition)+" > 50///////////////////////////////////////");
                    return true;
                } else {
                    return false;
                }
            }
        }
        public Action actionClawClose(){
            return new ActionClawClose();
        }

    }

}





