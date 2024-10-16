package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "AUTO_LEFT_LINEAR", group = "Autonomous")
public class AutoLeftLinear extends LinearOpMode {

    double SOUTH = Math.PI;
    double EAST = 3 * Math.PI / 2;
    double NORTH = 0;
    double WEST = Math.PI / 2;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-29.5, -65, WEST);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ActionLib.RobotLift             lift            = new ActionLib.RobotLift(hardwareMap);
//        ActionLib.RobotIntakeSlide      intakeSlide     = new ActionLib.RobotIntakeSlide(hardwareMap);
//        ActionLib.RobotIntakeRotator    intakeRotator   = new ActionLib.RobotIntakeRotator(hardwareMap);
//        ActionLib.RobotIntakeClaw       intakeClaw      = new ActionLib.RobotIntakeClaw(hardwareMap);

        TrajectoryActionBuilder leftPathToSub = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-4, -29), WEST); //spline out to the sub

        TrajectoryActionBuilder leftPathSegregationWHOOOOOO1 = drive.actionBuilder(initialPose)
                .lineToYConstantHeading(-35)//back up from the sub
                .splineToConstantHeading(new Vector2d(-47,-44 ), WEST); //first pixel alignment
                //pick up first pixel
        TrajectoryActionBuilder leftPathSegregationWHOOOOOO2 = drive.actionBuilder(initialPose)
                .turnTo(EAST)
                .waitSeconds(0.01)
                .splineToConstantHeading(new Vector2d(-63,-54), EAST); //first basket
                //drop first pixel
        TrajectoryActionBuilder leftPathSegregationWHOOOOOO3 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-60,-54)) //move away from wall for turn
                .turnTo(WEST)
                .splineToConstantHeading(new Vector2d(-56,-44),WEST) //second pixel alignment
                .turnTo(EAST);
                //pick up second pixel
        TrajectoryActionBuilder leftPathSegregationWHOOOOOO4 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-63,-54), EAST);//second basket
                //drop second pixel
        TrajectoryActionBuilder leftPathSegregationWHOOOOOO5 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-58, -54)) //move away from wall for turn
                .turnTo(NORTH)
                .splineToConstantHeading(new Vector2d(-24,-6),NORTH);//go to sub
                //ascend here

        waitForStart();
        if (isStopRequested()) return;

        Action trajectoryActionToSub = leftPathToSub.build();
        Action trajectoryActionSeg1 = leftPathSegregationWHOOOOOO1.build();
        Action trajectoryActionSeg2 = leftPathSegregationWHOOOOOO2.build();
        Action trajectoryActionSeg3 = leftPathSegregationWHOOOOOO3.build();
        Action trajectoryActionSeg4 = leftPathSegregationWHOOOOOO4.build();
        Action trajectoryActionSeg5 = leftPathSegregationWHOOOOOO5.build();

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionToSub,
                        lift.liftSpecimen(),
                        lift.liftDown(),
                        trajectoryActionSeg1,
                        trajectoryActionSeg2,
                        trajectoryActionSeg3,
                        trajectoryActionSeg4,
                        trajectoryActionSeg5
                )
        );
    }
}
