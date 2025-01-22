package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.MecanumDrive;

import java.util.Arrays;

@Config
@Autonomous(name = "AUTO_RIGHT_LINEAR_4_SPECIMEN", group = "Autonomous")
public class AutoRightLinearHighPointHang4 extends LinearOpMode {

    double SOUTH = Math.PI;
    double EAST = 3 * Math.PI / 2;
    double NORTH = 0;
    double NORTH_EAST = 7 *Math.PI /4;
    double WEST = Math.PI / 2;
    double SOUTHEAST = 1.25 * Math.PI;


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
                .splineToConstantHeading(new Vector2d(10, -32.5), WEST); //spline out to the sub and scoring first X was 7
        Action trajectoryActionToSub = rightPathToSub.build();


        initialPose = new Pose2d(10, -32.5, WEST);// driving to the first sample
        TrajectoryActionBuilder rightPathDropSamples = drive.actionBuilder(initialPose)
//                .waitSeconds(0.125)
                .lineToY(-37)
                .strafeToLinearHeading(new Vector2d(58,-44),Math.PI / 2 + 0.01745329);
        Action trajectoryActionSlideToSample = rightPathDropSamples.build();

        initialPose = new Pose2d(58, -44, Math.PI / 2 + 0.01745329);// driving to capture the first sample
        TrajectoryActionBuilder rightPathFirstSampleNear = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(57,-39));
        Action trajectoryActionFirstSampleNear = rightPathFirstSampleNear.build();

        initialPose = new Pose2d(57, -39, WEST);
        Vector2d thaPose = initialPose.position;
        VelConstraint baseVelConstraint = new MinVelConstraint(Arrays.asList(
                new TranslationalVelConstraint(30.0),
                new AngularVelConstraint(2.0))//3
        );

        TrajectoryActionBuilder FirstSampleDropOff = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(58, -46), ((3 * Math.PI / 2)+0.004363323), baseVelConstraint);
        Action trajectoryActionFirstSampleDropOff      = FirstSampleDropOff.build();

        initialPose = new Pose2d(58, -46, ((3 * Math.PI / 2)+0.004363323));// driving to capture the second sample
        TrajectoryActionBuilder rightPathLineUpForTheSecondSample = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(58,-10));
        Action trajectoryActionLineUpForTheSecondSample = rightPathLineUpForTheSecondSample.build();

        initialPose = new Pose2d(58, -10, EAST);// driving to capture the second sample
        TrajectoryActionBuilder rightPathLineUpForTheSecondSample2 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(47,-14));//X was 46
        Action trajectoryActionLineUpForTheSecondSample2 = rightPathLineUpForTheSecondSample2.build();

        initialPose = new Pose2d(47, -14, EAST);// driving to capture the second sample
        TrajectoryActionBuilder rightPathPushingSecondSample = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(46,-48.75));
        Action trajectoryActionRightPathPushingSecondSample = rightPathPushingSecondSample.build();


        initialPose = new Pose2d(46, -48.75, EAST);// driving to capture the second sample
        TrajectoryActionBuilder rightPathScoreSample = drive.actionBuilder(initialPose)
                .lineToY(-45)
                .strafeToLinearHeading(new Vector2d(2,-28),WEST);// scoring second specimen
        Action trajectoryActionRightPathScoreSample = rightPathScoreSample.build();

        initialPose = new Pose2d(2, -29, WEST);// driving to capture the second sample
        TrajectoryActionBuilder rightPathPickUpSecondSample = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(26,-58),0-0.02181662);
        Action trajectoryActionrightPathPickUpSecondSample = rightPathPickUpSecondSample.build();

        initialPose = new Pose2d(26, -58, 0-0.02181662);// driving to capture the second sample
        TrajectoryActionBuilder rightPathPickUpSecondSample2 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(31.5,-58));
        Action trajectoryActionrightPathPickUpSecondSample2 = rightPathPickUpSecondSample2.build();


        initialPose = new Pose2d(31.5, -58, 0-0.02181662);//was 59.5
        TrajectoryActionBuilder rightPathScoreSample2 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(5,-29),WEST);//scoring third specimen
        Action trajectoryActionRightPathScoreSample2 = rightPathScoreSample2.build();

        initialPose = new Pose2d(4, -28.5, WEST);
        TrajectoryActionBuilder ToThirdSample = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(26,-56),0-0.03490659);//NORTH
        Action trajectoryActionToThirdSample = ToThirdSample.build();

        initialPose = new Pose2d(26, -56, 0-0.03490659);//NORTH
        TrajectoryActionBuilder ToThirdSample2 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(31.5,-56),0-0.03490659);//NORTH
        Action trajectoryActionToThirdSample2 = ToThirdSample2.build();

        initialPose = new Pose2d(31.5, -56, 0-0.03490659);//NORTH
        TrajectoryActionBuilder ToDriveToBasket1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(6,-29.5),Math.PI / 2+0.03490659);
                //.lineToY(-29.5);//scoring fourth specimen
        Action trajectoryActionToDriveToBasket1 = ToDriveToBasket1.build();

        initialPose = new Pose2d(6, -29.5, WEST);
        TrajectoryActionBuilder ToDriveToBasket2 = drive.actionBuilder(initialPose)
                .strafeToConstantHeading(new Vector2d(5,-60));
        Action trajectoryActionToDriveToBasket2 = ToDriveToBasket2.build();

// wait section claw down
        TrajectoryActionBuilder wait = drive.actionBuilder(initialPose)
                .waitSeconds(2.3);// was 2.25
        Action trajectoryActionWait1 = wait.build();

        TrajectoryActionBuilder wait2 = drive.actionBuilder(initialPose)
                .waitSeconds(0.15);
        Action trajectoryActionWait2 = wait2.build();

        TrajectoryActionBuilder wait3 = drive.actionBuilder(initialPose)
                .waitSeconds(0.25);
        Action trajectoryActionWait3 = wait3.build();

        TrajectoryActionBuilder wait4 = drive.actionBuilder(initialPose)
                .waitSeconds(0.45);
        Action trajectoryActionWait4 = wait4.build();

        TrajectoryActionBuilder wait5 = drive.actionBuilder(initialPose)
                .waitSeconds(0.85);
        Action trajectoryActionWait5 = wait5.build();

        TrajectoryActionBuilder wait8 = drive.actionBuilder(initialPose)
                .waitSeconds(0.85);
        Action trajectoryActionWait8 = wait8.build();

        TrajectoryActionBuilder wait6 = drive.actionBuilder(initialPose)
                .waitSeconds(0.15);
        Action trajectoryActionWait6 = wait6.build();

        TrajectoryActionBuilder wait7 = drive.actionBuilder(initialPose)
                .waitSeconds(0.15);
        Action trajectoryActionWait7 = wait7.build();

        TrajectoryActionBuilder wait9 = drive.actionBuilder(initialPose)
                .waitSeconds(0.15);
        Action trajectoryActionWait9 = wait9.build();



// wait section claw up

        TrajectoryActionBuilder waitForClaw1 = drive.actionBuilder(initialPose)
                .waitSeconds(0.15);
        Action trajectoryActionwaitForClaw1 = waitForClaw1.build();

        TrajectoryActionBuilder waitForClaw2 = drive.actionBuilder(initialPose)
                .waitSeconds(0.05);
        Action trajectoryActionwaitForClaw2 = waitForClaw2.build();



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
                lift.actionLiftDown(),
                    new SequentialAction(
                    trajectoryActionWait1,
                    intakeSlide.actionReach(),
                    intakeClaw.actionClawOpen())
                ),
                trajectoryActionFirstSampleNear,
                intakeClaw.actionClawClose(),
                trajectoryActionWait2,
                new ParallelAction(
                        intakeSlide.actionRetract(),
                        trajectoryActionFirstSampleDropOff),

                intakeSlide.actionReach(),
                //trajectoryActionWait3,
                intakeClaw.actionClawOpen(),
                    new ParallelAction(
                            intakeSlide.actionRetract(),
                            trajectoryActionLineUpForTheSecondSample),
                trajectoryActionLineUpForTheSecondSample2,
                    new ParallelAction(
                            lift.actionLiftDown(),
                            trajectoryActionRightPathPushingSecondSample,
                    new SequentialAction(
                            trajectoryActionWait4,
                            intakeSlide.actionReach())),
                intakeClaw.actionClawClose(),
                trajectoryActionwaitForClaw1,

                new ParallelAction(
                    intakeSlide.actionRetract(),
                    trajectoryActionRightPathScoreSample,
                    lift.actionLiftSpecimen()),

                lift.actionliftScore(),
                intakeClaw.actionClawOpen(),
                new ParallelAction(
                        lift.actionLiftDown(),
                        trajectoryActionrightPathPickUpSecondSample,
                        new SequentialAction(
                                trajectoryActionWait5,
                                intakeSlide.actionReach())
                ),trajectoryActionrightPathPickUpSecondSample2,
                intakeClaw.actionClawClose(),
                trajectoryActionWait6,
                new ParallelAction(intakeSlide.actionRetract(),trajectoryActionRightPathScoreSample2,lift.actionLiftSpecimen()),
                lift.actionliftScore(),
                intakeClaw.actionClawOpen(),
                new ParallelAction(
                        lift.actionLiftDown(),
                        trajectoryActionToThirdSample,
                        new SequentialAction(
                                trajectoryActionWait8,
                                intakeSlide.actionReach())
                ),
                trajectoryActionToThirdSample2,
                intakeClaw.actionClawClose(),
                trajectoryActionWait9,
                new ParallelAction(
                        intakeSlide.actionRetract(),
                        trajectoryActionToDriveToBasket1,
                        lift.actionLiftSpecimen()
                ),
                lift.actionliftScore(),
                intakeClaw.actionClawOpen(),
                lift.actionLiftDown(),
                trajectoryActionToDriveToBasket2,
                intakeSlide.actionRetract()




            )
        );
    }
}
