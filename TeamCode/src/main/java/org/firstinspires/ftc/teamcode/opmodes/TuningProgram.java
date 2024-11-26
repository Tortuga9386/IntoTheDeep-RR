package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Disabled
@Autonomous(name = "Tuning", group = "Autonomous")
public class TuningProgram extends LinearOpMode {

    double SOUTH = Math.PI;
    double EAST = 3 * Math.PI / 2;
    double NORTH = 0;
    double NORTH_EAST = 7 *Math.PI /4;
    double WEST = Math.PI / 2;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(0, -0, WEST);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ActionLib.RobotLift             lift            = new ActionLib.RobotLift(hardwareMap);
        ActionLib.RobotIntakeSlide      intakeSlide     = new ActionLib.RobotIntakeSlide(hardwareMap);
//        ActionLib.RobotIntakeRotator    intakeRotator   = new ActionLib.RobotIntakeRotator(hardwareMap);
        ActionLib.RobotIntakeClaw       intakeClaw      = new ActionLib.RobotIntakeClaw(hardwareMap);

        //Init robot position
        intakeClaw.clawClose();

        TrajectoryActionBuilder rightPathToSub = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(24,0))
//                .waitSeconds(0.01)
//                .splineToConstantHeading(new Vector2d(36,-24 ), WEST)
//                .turnTo(NORTH) //turn 90 degrease
//                .strafeToConstantHeading(new Vector2d(46,-24))
//                .turnTo(NORTH_EAST)
//                .strafeToLinearHeading(new Vector2d(50,-60),EAST)
//                .waitSeconds(0.001)
//                .strafeToConstantHeading(new Vector2d(50,-12))
//                .splineToConstantHeading(new Vector2d(56,-20),EAST)
//                .waitSeconds(0.01)
//                .strafeToConstantHeading(new Vector2d(56,-60))
//                .waitSeconds(0.01)
//                .strafeToConstantHeading(new Vector2d(56,-12))
//                .splineToConstantHeading(new Vector2d(63,-15),EAST)
//                .waitSeconds(0.01)
//                .strafeToConstantHeading(new Vector2d(63,-60))
                ;

        TrajectoryActionBuilder wait = drive.actionBuilder(initialPose)
                .waitSeconds(0.5);

        Action trajectoryActionToSub = rightPathToSub.build();
        telemetry.addData("Status", "> INIT");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
            new SequentialAction(
                new ParallelAction(
                    trajectoryActionToSub
//                    new SequentialAction( lift.actionLiftSpecimen() )
//                ),
//                lift.actionliftScore(),
//                intakeClaw.actionClawOpen(),
//                lift.actionLiftDown(),
//                new ParallelAction(
//                    trajectoryActionDropSamples,
//                    new SequentialAction( lift.actionLiftDown() )
                )
            )
        );
    }
}
