package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name="***TeleOp***", group="teleop")
public class MainTeleOp extends RobotBase
{

    private   ElapsedTime   runtime = new ElapsedTime();

    public MainTeleOp() {}

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        super.INITIALIZE_DRIVE  = true;
        super.init();

        //Set initial positions
        telemetry.addData("Status", "init complete");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //telemetry.addData("Status", "Run Time:" + runtime.toString());
        //Log.v("Loop", "Run Time:" + runtime.toString());

        drive_loop();
        lift_loop();
        husky_loop();

//        telemetry.addData("1", "?>" + "");
//        telemetry.addData("2", "?>" + "");
//        telemetry.addData("3", "?>" + "");
//        telemetry.addData("4", "?>" + "");
    }


    protected void lift_loop() {

        double clawTarget;
        if (gamepad2.left_trigger > 0.01) {
            clawTarget = 0.015 + 0.075 * gamepad2.left_trigger;
            lift.claw.servo.position(clawTarget);
        } else {
            lift.claw.servo.close();
        }
    }

    protected void drive_loop() {

        //*** Test for manual override/turtle mode/field centric ***
        telemetry.addData("left_stick_x:", gamepad1.left_stick_x);
        telemetry.addData("left_stick_y:", gamepad1.left_stick_y);
        telemetry.addData("right_stick_x:", gamepad1.right_stick_x);
        telemetry.addData("right_stick_y:", gamepad1.right_stick_y);
        telemetry.addData("gamepad2.left_trigger:", gamepad2.left_trigger);
        telemetry.addData("gamepad2.right_trigger:", gamepad2.right_trigger);

        drive.turtleFactor = (1-gamepad1.right_trigger*0.5);
        telemetry.addData("turtleFactor:", drive.turtleFactor);
        drive.driveFromGamepad(gamepad1);

        if (gamepad1.right_stick_button) {
            drive.turn();
        }
    }

    protected void husky_loop() {
        sensorHuskyLens.setAlgorithm("COLOR");
        sensorHuskyLens.getColorBlocks();
    }

    protected void imu_loop() {

    }

    protected void telemetry_loop() {

    }

}
