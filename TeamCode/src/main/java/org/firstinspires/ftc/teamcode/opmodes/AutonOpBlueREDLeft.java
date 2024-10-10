package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
import org.firstinspires.ftc.teamcode.tuning.TuningOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="1) LEFT", group = "auto")
public final class AutonOpBlueREDLeft extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, -65, 90);
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            //.splineTo(new Vector2d(30, 30), Math.PI / 2)
                            //.splineTo(new Vector2d(0, 0), Math.PI / 2)

                            //.setTangent(0)
                            //.splineToConstantHeading(new Vector2d(30, 30), Math.PI / 2)
                            .splineToConstantHeading(new Vector2d(0, -40), 90)

                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
