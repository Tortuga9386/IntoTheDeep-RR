package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="1) RIGHT", group = "auto")
public final class AutonOpBlueREDright extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(18, -65, 90);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            //.splineToConstantHeading(new Vector2d(0, -24), 0)
                            .splineToConstantHeading(new Vector2d(-18, -65), 90)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
