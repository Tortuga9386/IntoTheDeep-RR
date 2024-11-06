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
@Autonomous(name = "AUTO_LEFT_LINEAR", group = "Autonomous")
public class AutoLeftLinear extends LinearOpMode {

    double SOUTH = Math.PI;
    double EAST = 3 * Math.PI/2 ;
    double SOUTHEAST = 1.25 * Math.PI;
    double NORTH = 0;
    double WEST = Math.PI / 2;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(-29.5, -65, WEST);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ActionLib.RobotLift             lift            = new ActionLib.RobotLift(hardwareMap);
        ActionLib.RobotIntakeSlide      intakeSlide     = new ActionLib.RobotIntakeSlide(hardwareMap);
//        ActionLib.RobotIntakeRotator    intakeRotator   = new ActionLib.RobotIntakeRotator(hardwareMap);
        ActionLib.RobotIntakeClaw       intakeClaw      = new ActionLib.RobotIntakeClaw(hardwareMap);

        //Init robot position
        intakeClaw.clawClose();

        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Go to the sub to deposit specimen
        initialPose = new Pose2d(-29.5, -65, WEST);
        TrajectoryActionBuilder pathSub = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-4, -32.75), WEST); //spline out to the sub
        Action trajectorySub = pathSub.build();

        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Go to the first sample - near
        initialPose = new Pose2d(-4, -32.75, WEST);
        TrajectoryActionBuilder pathSampleOneNear = drive.actionBuilder(initialPose)
                .lineToYConstantHeading(-36)//back up from the sub
                .strafeToConstantHeading(new Vector2d(-48, -47)); //strafe to the first sample - near
        Action trajectorySampleOneNear      = pathSampleOneNear.build();

        //TESTING PATH//////////////////////////////////////////////////////////////////////////////////////////////////
        //Go to the first sample - near
        initialPose = new Pose2d(-4, -32.75, WEST);
        TrajectoryActionBuilder pathSampleOneNearFAST = drive.actionBuilder(initialPose)
                .lineToYConstantHeading(-36)//back up from the sub
                .strafeToConstantHeading(new Vector2d(-48, -42)); //strafe to the first sample - near
                //.waitSeconds(0.01);
                //.strafeToConstantHeading(new Vector2d(-48, -43));
        Action trajectorySampleOneNearFAST      = pathSampleOneNearFAST.build();

        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Go to the first sample - close
        initialPose = new Pose2d(-48, -47, WEST);
        TrajectoryActionBuilder pathSampleOneClose = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-48, -43)) //strafe to the first sample - close
                .waitSeconds(0.1);
        Action trajectorySampleOneClose      = pathSampleOneClose.build();

        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Go to the scoring basket - near 1
        initialPose = new Pose2d(-48, -43, WEST);
        TrajectoryActionBuilder pathBasketNearTurn1 = drive.actionBuilder(initialPose)
                .turnTo(SOUTHEAST);

                //.splineToSplineHeading(new Pose2d(-50,-50, WEST), SOUTHEAST); //first basket
        Action trajectoryPathBasketNearTurn1      = pathBasketNearTurn1.build();

        //TESTING//////////////////////////////////////////////////////////////////////////////////////////////////
        //Go to the scoring basket - near 1
        initialPose = new Pose2d(-48, -43, WEST);
        TrajectoryActionBuilder pathBasketNear1 = drive.actionBuilder(initialPose)
                //.strafeTo(new Vector2d(-50, -50));
            .splineToSplineHeading(new Pose2d(-50,-50, WEST), SOUTHEAST); //first basket
        Action trajectoryPathBasketNear1      = pathBasketNear1.build();

        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Go to the scoring basket - near 2
        initialPose = new Pose2d(-55.5, -47, WEST);
        TrajectoryActionBuilder pathBasketNear2 = drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(-50,-50), SOUTHEAST);
        Action trajectoryPathBasketNear2      = pathBasketNear2.build();

        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Go to the scoring basket - close
        initialPose = new Pose2d(-50, -50, SOUTHEAST);
        TrajectoryActionBuilder pathBasketClose = drive.actionBuilder(initialPose)
                //.waitSeconds(0.50)
                .strafeToConstantHeading(new Vector2d(-55, -55));//Move into scoring position
        Action trajectoryPathBasketClose      = pathBasketClose.build();

        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Back up from the scoring basket
        initialPose = new Pose2d(-55, -55, SOUTHEAST);
        TrajectoryActionBuilder pathBasketNearBackup = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-45, -45));//Move into scoring position
        Action trajectoryPathBasketNearBackup      = pathBasketNearBackup.build();


        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Go to the second sample - near
        initialPose = new Pose2d(-45, -45, SOUTHEAST);
        TrajectoryActionBuilder pathSampleTwoNear = drive.actionBuilder(initialPose)
                //.splineToConstantHeading(new Vector2d(-47, -47)); //strafe to the first sample - near
               // .splineToConstantHeading(new Vector2d(-4, -32.75), WEST); //spline out to the sub
        .splineTo(new Vector2d(-55.5, -47), WEST);
        Action trajectorySampleTwoNear      = pathSampleTwoNear.build();

        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Go to the second sample - close
        initialPose = new Pose2d(-55.5, -47, WEST);
        TrajectoryActionBuilder pathSampleTwoClose = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-55.5, -43)); //strafe to the first sample - close
        Action trajectorySampleTwoClose      = pathSampleTwoClose.build();


        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Wait action
        initialPose = new Pose2d(-60, -55, EAST);
        TrajectoryActionBuilder pathWait = drive.actionBuilder(initialPose)
                .waitSeconds(0.3);
        Action trajectoryWait      = pathWait.build();


        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                    //Drive to sub and score specimen
                    new ParallelAction(
                        trajectorySub,
                        new SequentialAction( lift.actionLiftSpecimen() )
                    ),

                    //Score the specimen
                    lift.actionliftScore(),
                    intakeClaw.actionClawOpen(),
                    lift.actionLiftDown(),

                    //Drive to to the first sample
                    new ParallelAction(
                        trajectorySampleOneNearFAST,
                        //trajectorySampleOneNear,
                        intakeClaw.actionClawOpen(),
                        lift.actionClawGrab()
                    ),

                    //Collect the sample
                    intakeSlide.actionReach(), //Lowers the intake slide
                    trajectorySampleOneClose,
                    intakeClaw.actionClawClose(), //Grabs the specimen
                    intakeSlide.actionRetract(), //Raises the intake slide

                    //Drive to sample basket with first sample
                    new ParallelAction(
                        //trajectoryPathBasketNearTurn1,
                        trajectoryPathBasketNear1,
                        lift.actionLiftToBasket()
                    ),

                    //Score the sample
                    trajectoryPathBasketClose,
                    intakeClaw.actionClawOpen(),
                    trajectoryPathBasketNearBackup,
                    lift.actionLiftDown(),

                    //Drive to the second sample
                    new ParallelAction(
                            trajectorySampleTwoNear,
                            intakeClaw.actionClawOpen(),
                            lift.actionClawGrab(),
                            intakeSlide.actionReach()
                    ),

                    //Collect the sample
                    trajectorySampleTwoClose,
                    intakeClaw.actionClawClose(), //Grabs the specimen
                    intakeSlide.actionRetract(), //Raises the intake slide

                    // Drive to sample basket with second sample
                    new ParallelAction(
                        trajectoryPathBasketNear2,
                        lift.actionLiftToBasket()
                    ),

                    //Score the sample
                    trajectoryPathBasketClose,
                    intakeClaw.actionClawOpen(),
                    trajectoryPathBasketNearBackup,
                    lift.actionLiftDown()


            )
        );

    }
}
