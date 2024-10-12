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
        lift.slide.doSlideStuff(gamepad2);
//        telemetry.addData("GP2.left_stick_y:", gamepad2.left_stick_y);
//        telemetry.addData("GP2.a:", gamepad2.a);
//        telemetry.addData("GP2.b:", gamepad2.b);
//        telemetry.addData("GP2.x:", gamepad2.x);
//        telemetry.addData("GP2.y:", gamepad2.y);
    }

    protected void drive_loop() {

        //
        //slidePosition
        drive.turtleFactor = (1 - 0.75 * lift.slide.slideMotorCenter.getCurrentPosition() / 5000);
//        telemetry.addData("turtleFactor:", drive.turtleFactor);
        drive.driveFromGamepad(gamepad1);
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
