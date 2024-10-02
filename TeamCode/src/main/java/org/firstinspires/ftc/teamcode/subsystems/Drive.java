package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.opmodes.RobotBase;

public class Drive {

    //Inherited data objects
    protected HardwareMap hardwareMap;
    public Telemetry telemetry;
    protected RobotBase robotBase;

    //Motor object definitions
    public DcMotor leftFrontMotor;
    public DcMotor rightFrontMotor;
    public DcMotor leftRearMotor;
    public DcMotor rightRearMotor;

    //Turtle Mode
    public boolean turtleMode = false;
    public double  turtleFactor = 1;

    private final ElapsedTime time = new ElapsedTime();

    public Drive(HardwareMap hardwareMap, RobotBase opMode) {
        this.hardwareMap = hardwareMap;
        this.robotBase = opMode;
        this.telemetry = robotBase.telemetry;

        initHardware();
    }

    protected void initHardware() {
        try {
            leftFrontMotor = hardwareMap.get(DcMotorEx.class, "frontLeft");
        } catch (Exception e) {
            Log.v("Drive", ":leftFrontMotor init failed");
        }

        try {
            leftRearMotor = hardwareMap.get(DcMotorEx.class, "backLeft");
        } catch (Exception e){
            Log.v("Drive", ":leftRearMotor init failed");
        }

        try {
            rightFrontMotor = hardwareMap.get(DcMotorEx.class, "frontRight");
            rightFrontMotor.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception e){
            Log.v("Drive", ":rightFrontMotor init failed");
        }

        try {
            rightRearMotor = hardwareMap.get(DcMotorEx.class, "backRight");
            rightRearMotor.setDirection(DcMotor.Direction.REVERSE);
        } catch (Exception e){
            Log.v("Drive", ":rightRearMotor init failed");
        }
    }

    public void driveFromGamepad(Gamepad gamepad) {

        double deadZone = 0.1;

        double forward  = 0;
        if (Math.abs(gamepad.left_stick_y) > deadZone) {
            forward = gamepad.left_stick_y * turtleFactor;
        }
        Log.v("Drive", "left_stick_y:"+gamepad.left_stick_y);
        Log.v("Drive", "forward:"+forward);

        double strafe = 0;
        if (Math.abs(gamepad.left_stick_x) > deadZone) {
            strafe = -gamepad.left_stick_x * turtleFactor;
        }
        Log.v("Drive", "left_stick_x:"+gamepad.left_stick_x);
        Log.v("Drive", "strafe:"+strafe);

        double twist  = 0;
        if (Math.abs(gamepad.right_stick_x) > deadZone) {
            twist = -(gamepad.right_stick_x * turtleFactor)/2;
        }
        Log.v("Drive", "right_stick_x:"+gamepad.right_stick_x);
        Log.v("Drive", "twist:"+twist);

        // You may need to multiply some of these by -1 to invert direction of
        // the motor.  This is not an issue with the calculations themselves.
        double[] speeds = {
                (forward + strafe + twist),
                (forward - strafe - twist),
                (forward - strafe + twist),
                (forward + strafe - twist)
        };

        // Loop through all values in the speeds[] array and find the greatest
        // *magnitude*.  Not the greatest velocity.
        double max = Math.abs(speeds[0]);
        for(int i = 0; i < speeds.length; i++) {
            if ( max < Math.abs(speeds[i]) ) max = Math.abs(speeds[i]);
        }

        // If and only if the maximum is outside of the range we want it to be,
        // normalize all the other speeds based on the given speed value.
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        // apply the calculated values to the motors.
        leftFrontMotor.setPower(speeds[0]);
        rightFrontMotor.setPower(speeds[1]);
        leftRearMotor.setPower(speeds[2]);
        rightRearMotor.setPower(speeds[3]);
    }

    public void turn() {
        //time = new ElapsedTime();

    }

    public void turtleToggle() {
        if (turtleMode) {
            turtleMode = false;
            turtleFactor = 1;
        } else {
            turtleMode = true;
            turtleFactor = 0.5;
        }
    }

    public boolean isBusy() {
        return true;
    }

    public void stop() {
        leftFrontMotor.setPower(0);
        leftRearMotor.setPower(0);
        rightFrontMotor.setPower(0);
        rightRearMotor.setPower(0);
    }

    public void reset() {
        initHardware();
    }
}
