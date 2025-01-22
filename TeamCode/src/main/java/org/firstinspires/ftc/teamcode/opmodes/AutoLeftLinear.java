package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;
@Disabled
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
        Vector2d destinationVector;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ActionLib.RobotLift             lift            = new ActionLib.RobotLift(hardwareMap);
        ActionLib.RobotIntakeSlide      intakeSlide     = new ActionLib.RobotIntakeSlide(hardwareMap);
        ActionLib.RobotIntakeClaw       intakeClaw      = new ActionLib.RobotIntakeClaw(hardwareMap);
        ActionLib.RobotIntakeLinkage    intakeLinkage   = new ActionLib.RobotIntakeLinkage(hardwareMap);

        //Init robot position
        intakeClaw.clawClose();
        //intakeClaw.extendTheFinger();
        //intakeClaw.retractFinger();
//        intakeLinkage.linkageOut();

        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Go to the sub to deposit specimen
        initialPose = new Pose2d(-29.5, -65, WEST);
        TrajectoryActionBuilder pathSub = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-8, -32.75), WEST); //spline out to the sub
        Action trajectorySub = pathSub.build();

        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Go to the first sample - near
        initialPose = new Pose2d(-8, -32.75, WEST);
        TrajectoryActionBuilder pathSampleOneNear = drive.actionBuilder(initialPose)
                .lineToYConstantHeading(-36)//back up from the sub
                .strafeToConstantHeading(new Vector2d(-48, -46.5)); //strafe to the first sample - near
        Action trajectorySampleOneNear      = pathSampleOneNear.build();

        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Go to the first sample - close
        initialPose = new Pose2d(-48, -46.5, WEST);
        TrajectoryActionBuilder pathSampleOneClose = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-48, -41)); //strafe to the first sample - close
        Action trajectorySampleOneClose      = pathSampleOneClose.build();

        ////////////////////////////////////////////////////////////////////////////////////////////////////
        //Go to the scoring basket - near 1
        initialPose = new Pose2d(-48, -41, WEST);
        Vector2d thaPose = initialPose.position;
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(30.0),
                new AngularVelConstraint(3.0))
        );

        TrajectoryActionBuilder pathBasketNear1 = drive.actionBuilder(initialPose)
            .strafeToLinearHeading(new Vector2d(-50, -50), SOUTHEAST, baseVelConstraint);
        Action trajectoryPathBasketNear1      = pathBasketNear1.build();

        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Go to the scoring basket - close
        initialPose = new Pose2d(-50, -50, SOUTHEAST);
        TrajectoryActionBuilder pathBasketClose = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-54.0, -57.0)); //Move into scoring position
        Action trajectoryPathBasketClose      = pathBasketClose.build();

        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Back up from the scoring basket
        initialPose = new Pose2d(-54.0, -57.0, SOUTHEAST);
        TrajectoryActionBuilder pathBasketNearBackup = drive.actionBuilder(initialPose)
            //    .strafeToConstantHeading(new Vector2d(-45, -45));
            .strafeToLinearHeading(new Vector2d(-45, -45), WEST, baseVelConstraint);
        Action trajectoryPathBasketNearBackup      = pathBasketNearBackup.build();

        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Go to the second sample - near
        //initialPose = new Pose2d(-45, -45, WEST);
        initialPose = new Pose2d(-55.0, -57.0, SOUTHEAST);

        VelConstraint velConstraintSampleTwo = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(30.0),
                new AngularVelConstraint(3.0))
        );

        TrajectoryActionBuilder pathSampleTwoNear = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-56, -47.5), WEST, velConstraintSampleTwo); //Turn to grab sample 2
        Action trajectorySampleTwoNear      = pathSampleTwoNear.build();

        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Go to the second sample - close
        initialPose = new Pose2d(-56, -47.5, WEST);
        TrajectoryActionBuilder pathSampleTwoClose = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-56.0, -41)); //strafe to the second sample - close
        Action trajectorySampleTwoClose      = pathSampleTwoClose.build();

        ////////////////////////////////////////////////////////////////////////////////////////////////////
        //Go to the scoring basket - near 2
        initialPose = new Pose2d(-56.0, -41, WEST);

        VelConstraint baseVelConstraintBasket2 = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(30.0),
                new AngularVelConstraint(3.0))
        );

        TrajectoryActionBuilder pathBasketNear2 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(-50, -50), SOUTHEAST, baseVelConstraintBasket2);
        Action trajectoryPathBasketNear2      = pathBasketNear2.build();

        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Go to the scoring basket - close 2
        initialPose = new Pose2d(-50, -50, SOUTHEAST);
        TrajectoryActionBuilder pathBasketClose2 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(-53, -57)); //Move into scoring position
        Action trajectoryPathBasketClose2      = pathBasketClose2.build();

        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Back up from the scoring basket
//        initialPose = new Pose2d(-54.5, -56.5, SOUTHEAST);
//        TrajectoryActionBuilder pathBasketNearBackup2 = drive.actionBuilder(initialPose)
//                .strafeToConstantHeading(new Vector2d(-45, -45));
//        Action trajectoryPathBasketNearBackup2      = pathBasketNearBackup2.build();

        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Back up from the scoring basket
        initialPose = new Pose2d(-53, -57, SOUTHEAST);
        TrajectoryActionBuilder pathBasketNearBackup2 = drive.actionBuilder(initialPose)
            //.strafeToConstantHeading(new Vector2d(-19.5, -0.0));
            .strafeToLinearHeading(new Vector2d(-30, -10.0), SOUTH)//;
            .strafeToLinearHeading(new Vector2d(-26, -10.0), SOUTH);
        Action trajectoryPathBasketNearBackup2      = pathBasketNearBackup2.build();


        //////////////////////////////////////////////////////////////////////////////////////////////////
        //Wait action
        initialPose = new Pose2d(-48, -41, WEST);
        TrajectoryActionBuilder pathWait = drive.actionBuilder(initialPose)
                .waitSeconds(0.25);
        Action trajectoryWait      = pathWait.build();initialPose = new Pose2d(-48, -41, WEST);
        TrajectoryActionBuilder pathWait2 = drive.actionBuilder(initialPose)
                .waitSeconds(0.25);
        Action trajectoryWait2      = pathWait2.build();


        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(

                        //TEST LINKAGE ACTION, DO NOT USE IN PRODUCTION RUN//
                        //intakeSlide.actionReach(),
                        //intakeLinkage.actionLinkageOut()
                        //intakeClaw.actionClawClose(),
                        //intakeLinkage.actionLinkageIn()


                    //Drive to sub and score specimen
                    new ParallelAction(
                        trajectorySub,
                        new SequentialAction( lift.actionLiftSpecimen() )
                    ),

                    //Score the specimen
                    lift.actionliftScore(),
                    intakeClaw.actionClawOpen(),
                    //lift.actionLiftDown(),
                    lift.actionClawGrab(),

                    //Drive to to the first sample
                    new ParallelAction(
                        trajectorySampleOneNear      //Get near the sample before final approach
                        //intakeClaw.actionClawOpen(),  //Open the claw
                        //lift.actionClawGrab()         //Move lift to correct height
                    ),

                    //Collect the sample
                    intakeSlide.actionReach(),        //Lowers the intake slide
                    trajectorySampleOneClose,         //Move close to sample 1
                    intakeClaw.actionClawClose(),//Grabs the sample
                    trajectoryWait,
                    intakeSlide.actionRetract(),      //Raises the intake slide

                    //Drive to sample basket with first sample
                    trajectoryPathBasketNear1,        //Get near the basket before final approach
                    intakeSlide.actionRetract(),
                    new ParallelAction(
                    intakeSlide.actionRetract(),  //This is just to make sure the slide doesn't torque down when we lift
                    lift.actionLiftToBasket(),    //Move lift to basket scoring height
                    trajectoryPathBasketClose
                    ),

                    //Score the sample
                    intakeClaw.actionClawOpen(),
                    intakeSlide.actionRetract(),
                    lift.actionLiftDown(),

                    //Get away from basket
//                    new ParallelAction(
//                        //lift.actionLiftDown(),
//                        trajectoryPathBasketNearBackup
//                    ),
                    //trajectoryPathBasketNearBackup,

                    //Drive to to the second sample
                    new ParallelAction(
                        trajectorySampleTwoNear,      //Get near the sample before final approach
                        intakeClaw.actionClawOpen(),  //Open the claw
                        lift.actionClawGrab()         //Move lift to correct height
                    ),

                    //Collect the sample
                    intakeSlide.actionReach(),        //Lowers the intake slide
                    trajectorySampleTwoClose,         //Move close to sample 1
                    intakeClaw.actionClawClose(),//Grabs the sample
                    trajectoryWait2,
                    intakeSlide.actionRetract(),      //Raises the intake slide

                    //Drive to sample basket with second sample
                    trajectoryPathBasketNear2,        //Get near the basket before final approach
                    intakeSlide.actionRetract(),  //This is just to make sure the slide doesn't torque down when we lift
                        new ParallelAction(
                        lift.actionLiftToBasket(),    //Move lift to basket scoring height
                        trajectoryPathBasketClose2
                    ),

                    //Score second sample
                    intakeClaw.actionClawOpen(),
                    intakeSlide.actionRetract(),
                    lift.actionLiftDown(),

                    //Get away from basket
//                    new ParallelAction(
//                        lift.actionLiftDown(),
//                        trajectoryPathBasketNearBackup2
//                    )
                    trajectoryPathBasketNearBackup2,
                    intakeSlide.actionRetract()
////////////////////////////////////////WORKS DOWN TO HERE///////////////////////////////////////////////
                        //lift.actionClawGrab(),
                        //trajectorySampleTwoNear,
                        //trajectorySampleTwoClose
                        //intakeSlide.actionReach()
//                    //Drive to the second sample


            )
        );

    }
}
