package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Autonomous(name="Test Roadrunner Sequence", group = "auto")
public class TestRoadRunnerSequence extends RobotBase {

    //public static final Class<?> DRIVE_CLASS = MecanumDrive.class;
    private int iterationCounter = 0;

    public TestRoadRunnerSequence() {
    }

    @Override
    public void init() {

    }

    public void loop() {
        iterationCounter++;
        if (iterationCounter < 2) {
            Log.i("Test RR Seq", "--- Below execution count (" + iterationCounter + ") ---");
            Pose2d beginPose = new Pose2d(0, 0, 0);
            //if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            //.lineToX(25)
                            //.splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineToConstantHeading(new Vector2d(30, 30), 0)
                            .splineToConstantHeading(new Vector2d(0, 0), 0)
                            .build());
        } else if (iterationCounter <= 500) {
            Log.i("Test RR Seq", "--- Exceeded execution count ("+iterationCounter+") ---");
        }
    }

}
