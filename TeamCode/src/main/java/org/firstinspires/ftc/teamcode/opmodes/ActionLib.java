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
        private double liftMotorSpeed = 1;
        private int downTarget = 0;
        private int downTargetPosition = 0;

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

        public Action actionLiftUp() {
            return new ActionLiftUp();
        }

        /////LIFT DOWN
        public class ActionLiftDown implements Action {
            private boolean initialized = false;

            public ActionLiftDown(int liftPosition) {
                Log.v("ActionLiftDown", "START//liftPosition: "+liftPosition+"/////////////////////////////////////////////////");
                downTargetPosition = downTarget;
                if (liftPosition > 0) {
                    downTargetPosition = liftPosition;
                }
            }

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                //int targetPosition = 0;

                double currentPosition = lift.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if ((Math.abs(downTargetPosition - currentPosition) > motorPrecision) || !quitter) {
                    goToTarget(downTargetPosition, liftMotorSpeed);
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

        public Action actionLiftDown(int liftPosition) {
            return new ActionLiftDown(liftPosition);
        }
        public Action actionLiftDown() {
            return new ActionLiftDown(0);
        }

        /////LIFT SPECIMEN
        public class ActionLiftSpecimen implements Action {

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Log.v("ActionLiftSpecimen", "START/////////////////////////////////////////////////////////////////");
                int targetPosition = 1390;

                double currentPosition = lift.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if ((Math.abs(targetPosition - currentPosition) > motorPrecision) || !quitter) {
                    Log.v("ActionLiftSpecimen", "RUNNING//////targetPosition" + targetPosition + " vs currentPosition" + currentPosition + "////////////////////////////////////////");
                    Log.v("ActionLiftSpecimen", "RUNNING//////testVal:" + Math.abs(targetPosition - currentPosition) + " > 50///////////////////////////////////////");
                    goToTarget(targetPosition, liftMotorSpeed);
                    return true;
                } else {
                    if (quitter) {
                        quitter = true;
                        Log.v("ActionLiftSpecimen", "FINISH//////I QUIT!!!!////////////////////////////////////////");
                        Log.v("ActionLiftSpecimen", "FINISH//////testVal:" + Math.abs(targetPosition - currentPosition) + " < 50///////////////////////////////////////");
                        Log.v("ActionLiftSpecimen", "FINISH//////////////////////////////////////////////////////////////////");
                        return false;
                    }
                    Log.v("ActionLiftSpecimen", "FINISH//////ON TARGET BUT NOT A QUITTER!////////////////////////////////////////");
                    Log.v("ActionLiftSpecimen", "FINISH//////testVal:" + Math.abs(targetPosition - currentPosition) + " < 50///////////////////////////////////////");
                    Log.v("ActionLiftSpecimen", "FINISH//////////////////////////////////////////////////////////////////");
                    return true;
                }
            }
        }

        public Action actionLiftSpecimen() {
            return new ActionLiftSpecimen();
        }


        /////LIFT SPECIMEN
        public class ActionLiftScore implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                int targetPosition = 650;

                double currentPosition = lift.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if ((Math.abs(targetPosition - currentPosition) > motorPrecision) || !quitter) {
                    goToTarget(targetPosition, 0.75);
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

        public Action actionliftScore() {
            return new ActionLiftScore();
        }

        /////CLAW GRAB
        public class ActionClawGrab implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Log.v("ActionClawGrab", "START/////////////////////////////////////////////////////////////////");
                int targetPosition = 125;//nick dropped this 55 ticks

                double currentPosition = lift.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if ((Math.abs(targetPosition - currentPosition) > motorPrecision) || !quitter) {
                    goToTarget(targetPosition, liftMotorSpeed);
                    Log.v("ActionClawGrab", "RUNNING//////targetPosition" + targetPosition + " vs currentPosition" + currentPosition + "////////////////////////////////////////");
                    Log.v("ActionClawGrab", "RUNNING//////testVal:" + Math.abs(targetPosition - currentPosition) + " > 50///////////////////////////////////////");
                    return true;
                } else {
                    if (quitter) {
                        quitter = true;
                        Log.v("ActionClawGrab", "FINISH//////I QUIT!!!!////////////////////////////////////////");
                        Log.v("ActionClawGrab", "FINISH//////testVal:" + Math.abs(targetPosition - currentPosition) + " < 50///////////////////////////////////////");
                        Log.v("ActionClawGrab", "FINISH//////////////////////////////////////////////////////////////////");
                        return false;
                    } else {
                        Log.v("ActionClawGrab", "FINISH//////ON TARGET BUT NOT A QUITTER!////////////////////////////////////////");
                        Log.v("ActionClawGrab", "FINISH//////testVal:" + Math.abs(targetPosition - currentPosition) + " < 50///////////////////////////////////////");
                        Log.v("ActionClawGrab", "FINISH//////////////////////////////////////////////////////////////////");
                    }
                    return true;
                }
            }
        }

        public Action actionClawGrab() {
            return new ActionClawGrab();
        }

        /////LIFT TO BASKET
        public class ActionLiftToBasket implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Log.v("ActionLiftToBasket", "START/////////////////////////////////////////////////////////////////");
                int targetPosition = 4100;

                double currentPosition = lift.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if ((Math.abs(targetPosition - currentPosition) > motorPrecision) || !quitter) {
                    goToTarget(targetPosition, liftMotorSpeed);
                    Log.v("ActionLiftToBasket", "RUNNING//////targetPosition" + targetPosition + " vs currentPosition" + currentPosition + "////////////////////////////////////////");
                    Log.v("ActionLiftToBasket", "RUNNING//////testVal:" + Math.abs(targetPosition - currentPosition) + " > 50///////////////////////////////////////");
                    return true;
                } else {
                    if (quitter) {
                        quitter = true;
                        Log.v("ActionLiftToBasket", "FINISH//////I QUIT!!!!////////////////////////////////////////");
                        Log.v("ActionLiftToBasket", "FINISH//////testVal:" + Math.abs(targetPosition - currentPosition) + " < 50///////////////////////////////////////");
                        Log.v("ActionLiftToBasket", "FINISH//////////////////////////////////////////////////////////////////");
                        return false;
                    } else {
                        Log.v("ActionLiftToBasket", "FINISH//////ON TARGET BUT NOT A QUITTER!////////////////////////////////////////");
                        Log.v("ActionLiftToBasket", "FINISH//////testVal:" + Math.abs(targetPosition - currentPosition) + " < 50///////////////////////////////////////");
                        Log.v("ActionLiftToBasket", "FINISH//////////////////////////////////////////////////////////////////");
                    }
                    return true;
                }
            }
        }

        public Action actionLiftToBasket() {
            return new ActionLiftToBasket();
        }


        /////CANCEL ANY LIFT HOLD OVERRIDES
        public class ActionQuitLift implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                disableLiftHold();
                return false;
            }
        }

        public Action actionQuitLift() {
            return new ActionQuitLift();
        }

        public class ActionHoldLift implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                quitter = false;
                return false;
            }
        }

        public Action actionHoldLift() {
            return new ActionHoldLift();
        }

    }

//////////////////////////////////////////////////////////////////////////
///  INTAKE SLIDE
//////////////////////////////////////////////////////////////////////////

    public static class RobotIntakeSlide {
        private DcMotor intakeSlide;
        private double motorPower = 1;

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
                int targetPosition = -20;

                if (!initialized) {
                    goToTarget(targetPosition, motorPower);
                    initialized = true;
                }

                double currentPosition = intakeSlide.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if (Math.abs(targetPosition - currentPosition) > 10) {
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
                int targetPosition = 1050;

                if (!initialized) {
                    goToTarget(targetPosition, motorPower);
                    initialized = true;
                }

                double currentPosition = intakeSlide.getCurrentPosition();
                packet.put("liftPos", currentPosition);
                if (Math.abs(targetPosition - currentPosition) > 10) {
                    return true;
                } else {
                    intakeSlide.setPower(0);
                    return false;
                }
            }
        }

        public Action actionReach() {
            return new ActionReach();
        }
    }

//////////////////////////////////////////////////////////////////////////
///  ROBOT INTAKE
//////////////////////////////////////////////////////////////////////////

//    public static class RobotIntakeRotator {
//        private Servo intakeRotatorServo;
//        private double rightPosition = 0.7;
//        private double leftPosition = 0.3;
//
//        public RobotIntakeRotator(HardwareMap hardwareMap) {
//            intakeRotatorServo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "linkage");
//            intakeRotatorServo.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
//        }
//
//        public class RotateLeft implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//
//                if (!initialized) {
//                    intakeRotatorServo.setPosition(leftPosition);
//                    initialized = true;
//                }
//
//                double pos = intakeRotatorServo.getPosition();
//                packet.put("clawPos", pos);
//                if (pos >= leftPosition) {
//                    return true;
//                } else {
//                    return false;
//                }
//            }
//        }
//
//        public Action rotateLeft() {
//            return new RotateLeft();
//        }
//
//        public class RotateRight implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    intakeRotatorServo.setPosition(rightPosition);
//                    initialized = true;
//                }
//
//                double pos = intakeRotatorServo.getPosition();
//                packet.put("clawPos", pos);
//                if (pos <= rightPosition) {
//                    return true;
//                } else {
//                    return false;
//                }
//            }
//        }
//
//        public Action rotateRight() {
//            return new RotateRight();
//        }
//
//    }

//////////////////////////////////////////////////////////////////////////
///  INTAKE CLAW
//////////////////////////////////////////////////////////////////////////


    public static class RobotIntakeClaw {
        private Servo intakeClawServo;
        private double openPosition = 0.65;
        private double closedPosition = 0.45;

        private Servo  fingerServo;
        private double fingerInPosition = 0.0;
        private double fingerOutPosition = 1.0;

        public RobotIntakeClaw(HardwareMap hardwareMap) {
            intakeClawServo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "frontclaw");
            intakeClawServo.setDirection(Servo.Direction.FORWARD);
            fingerServo     = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "finger");
            fingerServo.setDirection(Servo.Direction.FORWARD);
        }

        public void clawOpen() {
            intakeClawServo.setPosition(openPosition);
        }

        public void clawClose() {
            intakeClawServo.setPosition(closedPosition);
        }

        public void extendTheFinger()  {
            fingerServo.setPosition(fingerOutPosition);
        }
        public void retractFinger()  {
            fingerServo.setPosition(fingerInPosition);
        }

        public class ActionClawOpen implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Log.v("ActionClawOpen", "START/////////////////////////////////////////////////////////////////");
                extendTheFinger();

                if (!initialized) {
                    intakeClawServo.setPosition(openPosition);
                    initialized = true;
                }

                double currentPosition = intakeClawServo.getPosition();
                packet.put("clawPos", currentPosition);
                if (Math.abs(openPosition - currentPosition) > servoPrecision) {
                    intakeClawServo.setPosition(openPosition);
                    Log.v("ActionClawOpen", "RUNNING//////targetPosition"+openPosition+ " vs currentPosition"+currentPosition+"////////////////////////////////////////");
                    Log.v("ActionClawOpen", "RUNNING//////testVal:"+Math.abs(openPosition - currentPosition)+" > "+servoPrecision+"///////////////////////////////////////");
                    return true;
                } else {
                    Log.v("ActionClawOpen", "DONE//////testVal:"+Math.abs(openPosition - currentPosition)+" > "+servoPrecision+"///////////////////////////////////////");
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

        public Action actionClawClose() {
            return new ActionClawClose();
        }

    }

//////////////////////////////////////////////////////////////////////////
///  INTAKE LINKAGE
//////////////////////////////////////////////////////////////////////////
//    public static class RobotIntakeLinkage {
//        private Servo  intakeLinkage;
//        private double inPosition = 0.025;
//        private double outPosition = 0.30;
//
//        public RobotIntakeLinkage(HardwareMap hardwareMap) {
//            intakeLinkage = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "linkage");
//            intakeLinkage.setDirection(Servo.Direction.REVERSE);
//        }
//
//        public class ActionLinkageIn implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                Log.v("ActionLinkageIn", "START/////////////////////////////////////////////////////////////////");
//
//                if (!initialized) {
//                    intakeLinkage.setPosition(inPosition);
//                    initialized = true;
//                }
//
//                double currentPosition = intakeLinkage.getPosition();
//                packet.put("linkagePos", currentPosition);
//                if (Math.abs(inPosition - currentPosition) > servoPrecision) {
//                    intakeLinkage.setPosition(inPosition);
//                    return true;
//                } else {
//                    return false;
//                }
//
//            }
//        }
//        public Action actionLinkageIn() {
//            return new ActionLinkageIn();
//        }
//
//        public class ActionLinkageOut implements Action {
//            private boolean initialized = false;
//
//            @Override
//            public boolean run(@NonNull TelemetryPacket packet) {
//                if (!initialized) {
//                    intakeLinkage.setPosition(outPosition);
//                    initialized = true;
//                }
//
//                double currentPosition = intakeLinkage.getPosition();
//                packet.put("linkagePos", currentPosition);
//                if (Math.abs(outPosition - currentPosition) > servoPrecision) {
//                    intakeLinkage.setPosition(outPosition);
//                    Log.v("ActionLinkageOut", "RUNNING//////targetPosition"+outPosition+ " vs currentPosition"+currentPosition+"////////////////////////////////////////");
//                    Log.v("ActionLinkageOut", "RUNNING//////testVal:"+Math.abs(outPosition - currentPosition)+" > 50///////////////////////////////////////");
//                    return true;
//                } else {
//                    Log.v("ActionLinkageOut", "DONE//////targetPosition"+outPosition+ " vs currentPosition"+currentPosition+"////////////////////////////////////////");
//                    return false;
//                }
//            }
//        }
//
//        public Action actionLinkageOut() {
//            return new ActionLinkageOut();
//        }
//
//    }

//////////////////////////////////////////////////////////////////////////
///  INTAKE LINKAGE
//////////////////////////////////////////////////////////////////////////


    public static class RobotIntakeLinkage {
        private Servo intakeLinkageServo;
        private double inPosition = 0.3;
        private double outPosition = 0.5;

        public RobotIntakeLinkage(HardwareMap hardwareMap) {
            intakeLinkageServo = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "linkage");
            intakeLinkageServo.setDirection(Servo.Direction.FORWARD);
        }

        public void linkageOut() {
            intakeLinkageServo.setPosition(outPosition);
        }

        public void linkageIn() {
            intakeLinkageServo.setPosition(inPosition);
        }

        public class ActionLinkageOut implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                Log.v("ActionLinkageOut2", "START/////////////////////////////////////////////////////////////////");

                if (!initialized) {
                    linkageOut();
                    //intakeLinkageServo.setPosition(outPosition);
                    initialized = true;
                }

                double currentPosition = intakeLinkageServo.getPosition();
                packet.put("linkagePos2", currentPosition);
                if (Math.abs(outPosition - currentPosition) > servoPrecision) {
                    //intakeLinkageServo.setPosition(outPosition);
                    linkageOut();
                    Log.v("ActionLinkageOut2", "RUNNING//////targetPosition:"+outPosition+ " vs currentPosition:"+currentPosition+"////////////////////////////////////////");
                    Log.v("ActionLinkageOut2", "RUNNING//////testVal:"+Math.abs(outPosition - currentPosition)+" > "+servoPrecision+"///////////////////////////////////////");
                    return true;
                } else {
                    Log.v("ActionLinkageOut2", "DONE//////testVal:"+Math.abs(outPosition - currentPosition)+" > "+servoPrecision+"///////////////////////////////////////");
                    return false;
                }

            }
        }

        public Action actionLinkageOut() {
            return new ActionLinkageOut();
        }

        public class ActionLinkageIn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeLinkageServo.setPosition(inPosition);
                    initialized = true;
                }

                double currentPosition = intakeLinkageServo.getPosition();
                packet.put("linkagePos2", currentPosition);
                if (Math.abs(inPosition - currentPosition) > servoPrecision) {
                    intakeLinkageServo.setPosition(inPosition);
                    Log.v("ActionLinkageIn2", "RUNNING//////targetPosition"+inPosition+ " vs currentPosition"+currentPosition+"////////////////////////////////////////");
                    Log.v("ActionLinkageIn2", "RUNNING//////testVal:"+Math.abs(inPosition - currentPosition)+" > "+servoPrecision+"///////////////////////////////////////");
                    return true;
                } else {
                    Log.v("ActionLinkageIn2", "DONE//////testVal:"+Math.abs(outPosition - currentPosition)+" > "+servoPrecision+"///////////////////////////////////////");
                    return false;
                }
            }
        }
        public Action actionLinkageIn() {
            return new ActionLinkageIn();
        }

    }


}





