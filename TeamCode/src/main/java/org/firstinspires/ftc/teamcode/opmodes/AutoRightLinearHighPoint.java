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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "AUTO_RIGHT_LINEAR_HIGH_POINT", group = "Autonomous")
public class AutoRightLinearHighPoint extends LinearOpMode {

    double SOUTH = Math.PI;
    double EAST = 3 * Math.PI / 2;
    double NORTH = 0;
    double NORTH_EAST = 7 *Math.PI /4;
    double WEST = Math.PI / 2;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(16.5, -65, WEST);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ActionLib.RobotLift             lift            = new ActionLib.RobotLift(hardwareMap);
        ActionLib.RobotIntakeSlide      intakeSlide     = new ActionLib.RobotIntakeSlide(hardwareMap);
//        ActionLib.RobotIntakeRotator    intakeRotator   = new ActionLib.RobotIntakeRotator(hardwareMap);
        ActionLib.RobotIntakeClaw       intakeClaw      = new ActionLib.RobotIntakeClaw(hardwareMap);

        //Init robot position
        intakeClaw.clawClose();

        TrajectoryActionBuilder rightPathToSub = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(8, -32.75), WEST); //spline out to the sub
        Action trajectoryActionToSub = rightPathToSub.build();


        initialPose = new Pose2d(8, -32.75, WEST);// driving to the first sample
        TrajectoryActionBuilder rightPathDropSamples = drive.actionBuilder(initialPose)
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(58,-44));
        Action trajectoryActionSlideToSample = rightPathDropSamples.build();

        initialPose = new Pose2d(58, -44, WEST);// driving to capture the first sample
        TrajectoryActionBuilder rightPathFirstSampleNear = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(58,-40));
        Action trajectoryActionFirstSampleNear = rightPathFirstSampleNear.build();

        initialPose = new Pose2d(58, -40, WEST);// turn to put in the observation zone
        TrajectoryActionBuilder rightPathFirstSampleDropOff = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(58,-46),((3 * Math.PI / 2)+0.00000000000005));
        Action trajectoryActionFirstSampleDropOff = rightPathFirstSampleDropOff.build();

        initialPose = new Pose2d(58, -46, EAST);// driving to capture the second sample
        TrajectoryActionBuilder rightPathLineUpForTheSecondSample = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(58,-14));
        Action trajectoryActionLineUpForTheSecondSample = rightPathLineUpForTheSecondSample.build();

        initialPose = new Pose2d(58, -14, EAST);// driving to capture the second sample
        TrajectoryActionBuilder rightPathLineUpForTheSecondSample2 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(47,-16));
        Action trajectoryActionLineUpForTheSecondSample2 = rightPathLineUpForTheSecondSample2.build();

        initialPose = new Pose2d(47, -16, EAST);// driving to capture the second sample
        TrajectoryActionBuilder rightPathPushingSecondSample = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(46,-50.25));
        Action trajectoryActionRightPathPushingSecondSample = rightPathPushingSecondSample.build();


        initialPose = new Pose2d(46, -50.25, EAST);// driving to capture the second sample
        TrajectoryActionBuilder rightPathScoreSample = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(6,-28),WEST);
        Action trajectoryActionRightPathScoreSample = rightPathScoreSample.build();

        initialPose = new Pose2d(6, -28, WEST);// driving to capture the second sample
        TrajectoryActionBuilder rightPathPickUpSecondSample = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(28,-59.5),NORTH);
        Action trajectoryActionrightPathPickUpSecondSample = rightPathPickUpSecondSample.build();

        initialPose = new Pose2d(28, -59.5, NORTH);// driving to capture the second sample
        TrajectoryActionBuilder rightPathPickUpSecondSample2 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(30,-59.5));
        Action trajectoryActionrightPathPickUpSecondSample2 = rightPathPickUpSecondSample2.build();


        initialPose = new Pose2d(30, -59.5, NORTH);// driving to capture the second sample
        TrajectoryActionBuilder rightPathScoreSample2 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(0,-28),WEST);
        Action trajectoryActionRightPathScoreSample2 = rightPathScoreSample2.build();

        initialPose = new Pose2d(0, -28, WEST);// driving to capture the second sample
        TrajectoryActionBuilder Park = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(60,-60));
        Action trajectoryActionPark = Park.build();

// wait section claw down
        TrajectoryActionBuilder wait = drive.actionBuilder(initialPose)
                .waitSeconds(2);
        Action trajectoryActionWait1 = wait.build();

        TrajectoryActionBuilder wait2 = drive.actionBuilder(initialPose)
                .waitSeconds(0.25);
        Action trajectoryActionWait2 = wait2.build();

        TrajectoryActionBuilder wait3 = drive.actionBuilder(initialPose)
                .waitSeconds(0.25);
        Action trajectoryActionWait3 = wait3.build();

        TrajectoryActionBuilder wait4 = drive.actionBuilder(initialPose)
                .waitSeconds(0.25);
        Action trajectoryActionWait4 = wait4.build();

        TrajectoryActionBuilder wait5 = drive.actionBuilder(initialPose)
                .waitSeconds(2);
        Action trajectoryActionWait5 = wait5.build();

        TrajectoryActionBuilder wait6 = drive.actionBuilder(initialPose)
                .waitSeconds(0.25);
        Action trajectoryActionWait6 = wait6.build();

        TrajectoryActionBuilder wait7 = drive.actionBuilder(initialPose)
                .waitSeconds(0.25);
        Action trajectoryActionWait7 = wait7.build();



// wait section claw up

        TrajectoryActionBuilder waitForClaw1 = drive.actionBuilder(initialPose)
                .waitSeconds(0.25);
        Action trajectoryActionwaitForClaw1 = waitForClaw1.build();



        telemetry.addData("Status", "> INIT");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
            new SequentialAction(
                new ParallelAction(
                    trajectoryActionToSub,
                    new SequentialAction( lift.actionLiftSpecimen() )
                ),
                lift.actionliftScore(),
                intakeClaw.actionClawOpen(),
                new ParallelAction(
                trajectoryActionSlideToSample,
                lift.actionClawGrab(),
                    new SequentialAction(
                    trajectoryActionWait1,
                    intakeSlide.actionReach(),
                    intakeClaw.actionClawOpen())
                ),
                trajectoryActionFirstSampleNear,
                intakeClaw.actionClawClose(),
                trajectoryActionWait2,
                intakeSlide.actionRetract(),
                trajectoryActionFirstSampleDropOff,
                intakeSlide.actionReach(),
                trajectoryActionWait3,
                intakeClaw.actionClawOpen(),
                    new ParallelAction(
                            intakeSlide.actionRetract(),
                            trajectoryActionLineUpForTheSecondSample),
                trajectoryActionLineUpForTheSecondSample2,
                    new ParallelAction(
                            intakeSlide.actionReach(),
                            lift.actionLiftDown(),
                            trajectoryActionRightPathPushingSecondSample),
                intakeClaw.actionClawClose(),
                trajectoryActionwaitForClaw1,
                intakeSlide.actionRetract(),
                new ParallelAction(
                    trajectoryActionRightPathScoreSample,
                    lift.actionLiftSpecimen()),

                lift.actionliftScore(),
                intakeClaw.actionClawOpen(),
                lift.actionLiftDown(),
                trajectoryActionWait4,
                new ParallelAction(
                        trajectoryActionrightPathPickUpSecondSample,
                        new SequentialAction(
                                trajectoryActionWait5,
                                intakeSlide.actionReach())
                ),trajectoryActionrightPathPickUpSecondSample2,
                intakeClaw.actionClawClose(),
                trajectoryActionWait6,
                intakeSlide.actionRetract(),
                new ParallelAction(trajectoryActionRightPathScoreSample2,lift.actionLiftSpecimen()),
                lift.actionliftScore(),
                intakeClaw.actionClawOpen(),
                lift.actionLiftDown(),
                trajectoryActionWait7,
                trajectoryActionPark




            )
        );
    }
}
