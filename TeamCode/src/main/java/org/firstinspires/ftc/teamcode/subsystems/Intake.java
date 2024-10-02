package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.RobotBase;

public class Intake {
    protected static HardwareMap   hardwareMap;
    protected Telemetry     telemetry;
    protected RobotBase     robotBase;

    public Roller       roller;
    public Servo        servo;
    public droneLauncher droneLauncher;

    public boolean      allowManualInterrupt = true;
    private double      rollerForwardSpeed = 0.9;
    private double      rollerReverseSpeed = 0.5;
    private double      servoUpPosition = 0.4;
    private double      servoDownPosition = -0.10;
    private double      servoTravelPosition = 0.25;
    private double      servoCameraPosition = 0.2;
    private double      servoDisruptPosition = 0.1;
    private double      servoHoldPosition = 0.225;
    private static double      droneLauncherPosition = 0.35;
    private static double      droneLauncherInPosition = 0;
    private static double      droneLauncherOutPosition=1;

    public Intake(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;

        initHardware();
    }

    protected void initHardware() {
        roller  = new Roller();
        servo   = new Servo();
        droneLauncher = new droneLauncher();
    }

    public boolean isBusy(){
        if (allowManualInterrupt == false) {
            if (roller.isBusy()) { return true; }
            if (servo.isBusy()) { return true; }
            if (droneLauncher.isBusy()) { return true; }
            allowManualInterrupt = true;
        }
        return false;
    }

    public void stop () {
        roller.stop();
        servo.stop();
        droneLauncher.stop();
    }

    public class Roller {
        private DcMotor rollerMotor1;

        public Roller() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        protected void initHardware() {
            rollerMotor1 = hardwareMap.get(DcMotor.class, "intake");
            rollerMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rollerMotor1.setDirection(DcMotor.Direction.FORWARD);
            rollerMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        }

        public void forward(double speed) {
            Log.v("IntakeRoller.Roller", "IntakeRoller action: intakeRoller.forward()");

            rollerMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rollerMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rollerMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
            rollerMotor1.setPower(speed);

        }
        public void forward() {
            forward(rollerForwardSpeed);
        }

        public void reverse(double speed) {
            Log.v("IntakeRoller.Roller", "IntakeRoller action: intakeRoller.reverse()");

            rollerMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rollerMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rollerMotor1.setDirection(DcMotorSimple.Direction.FORWARD);
            rollerMotor1.setPower(speed);

        }
        public void reverse() {
            reverse(rollerReverseSpeed);
        }

        public boolean isBusy() {
            return rollerMotor1.isBusy();
        }

        public void stop() {
            rollerMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            rollerMotor1.setPower(0);
            rollerMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public class Servo {
        private com.qualcomm.robotcore.hardware.Servo deployServoLeft;
        private com.qualcomm.robotcore.hardware.Servo deployServoRight;
        private boolean rollerUp = true;

        public Servo() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        protected void initHardware() {
            deployServoLeft = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "intakeServoLeft");
            deployServoRight = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "intakeServoRight");
            deployServoRight.setDirection(com.qualcomm.robotcore.hardware.Servo.Direction.REVERSE);
            up();
        }

        protected void position(double targetPosition) {
            deployServoLeft.setPosition(targetPosition);
            deployServoRight.setPosition(targetPosition);
            Log.v("Intake.Servo", "Setting servo position: " + targetPosition);
        }
        public void up(double targetPosition) {
            Log.v("Intake.Servo", "Setting servo position: up(" + targetPosition + ")");
            position(targetPosition);
        }
        public void up(){
            up(servoUpPosition);
        }
        public void down(double targetPosition) {
            Log.v("Intake.Servo", "Setting servo position: down(" + targetPosition + ")");
            position(targetPosition);
        }
        public void down(){
            down(servoDownPosition);
        }
        public void disrupt(double targetPosition){
            Log.v("Intake.Servo", "Setting servo position: disrupt(" + targetPosition + ")");
            position(targetPosition);
        }
        public void disrupt(){ disrupt(servoDisruptPosition); }

        public void droneLaunchPosition(){
            Log.v("Intake.Servo", "Setting servo position: droneLaunchPosition(" + droneLauncherPosition + ")");
            position(droneLauncherPosition);
        }
        public void setCameraPosition() {
            Log.v("Intake.Servo", "Setting servo position: servoCameraPosition(" + droneLauncherPosition + ")");
            position(servoCameraPosition);
        }
        public void servoHoldposition() {
            position(servoHoldPosition);
        }

        public void downSlow() {
            double servoSlowStep = 0.1;
            double currentPosition = deployServoLeft.getPosition();
            double nextStepPosition = currentPosition+servoSlowStep;
            telemetry.addData("Position updates", "Current["+currentPosition+"] Next ["+nextStepPosition+"]");
            if (nextStepPosition < servoDownPosition) {
                down(nextStepPosition);
            } else {
                down();
            }

        }

        public void travel() {
            Log.v("Intake.Servo", "Setting servo position: travel(" + servoTravelPosition + ")");
            position(servoTravelPosition);
        }

        public double getCurrentPosition() {
            return deployServoLeft.getPosition();
        }

        public boolean isBusy() {
            return false;
        }

        public void stop() {
            double currentServoPosition = deployServoLeft.getPosition();
            Log.v("Intake.Servo", "Setting servo position: stop(" + currentServoPosition + ")");
            position(currentServoPosition);
        }
    }
    public class droneLauncher {
        private com.qualcomm.robotcore.hardware.Servo droneLauncher;
        private boolean pinIn = true;

        public droneLauncher() { //HardwareMap hardwareMap, RobotBase opMode
            initHardware();
        }

        protected void initHardware() {
            droneLauncher = hardwareMap.get(com.qualcomm.robotcore.hardware.Servo.class, "droneLauncher");
            in();
        }

        public void toggle() {

            if (pinIn == true) {
                out();
                pinIn = false;
            } else {
                in();
                pinIn = true;
            }
        }
        public void position(double targetPosition) {
            droneLauncher.setPosition(targetPosition);
        }
        public void in(double targetPosition) {
            droneLauncher.setPosition(targetPosition);
        }
        public void in(){
            in(droneLauncherInPosition);
        }
        public void out(double targetPosition) {
            droneLauncher.setPosition(targetPosition);
        }
        public void out(){
            out(droneLauncherOutPosition);
        }
        public void disrupt(double targetPosition){droneLauncher.setPosition(targetPosition);}
        public void disrupt(){disrupt(droneLauncherPosition);}
        public void setCameraPosition() { droneLauncher.setPosition(servoCameraPosition); }

        public void downSlow() {
            double servoSlowStep = 0.1;
            double currentPosition = droneLauncher.getPosition();
            double nextStepPosition = currentPosition+servoSlowStep;
            telemetry.addData("Position updates", "Current["+currentPosition+"] Next ["+nextStepPosition+"]");
            if (nextStepPosition < droneLauncherOutPosition) {
                out(nextStepPosition);
            } else {
                out();
            }

        }

        public void travel() {
            droneLauncher.setPosition(0.5);
        }

        public double getCurrentPosition() {
            return droneLauncher.getPosition();
        }

        public boolean isBusy() {
            return false;
        }

        public void stop() {
            droneLauncher.setPosition(droneLauncher.getPosition());
        }
    }
}