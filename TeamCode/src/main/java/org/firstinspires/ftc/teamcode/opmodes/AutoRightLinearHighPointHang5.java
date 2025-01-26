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
@Autonomous(name = "AUTO_RIGHT_5_SPEC", group = "Autonomous")
public class AutoRightLinearHighPointHang5 extends LinearOpMode {

    double SOUTH = Math.PI;
    double EAST = 3 * Math.PI / 2;
    double NORTH = 0;
    double NORTH_EAST = 7 *Math.PI /4;
    double NORTH_WEST = Math.PI/4;
    double WEST = Math.PI / 2;
    double SOUTHEAST = 1.25 * Math.PI;


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(6.25, -65, WEST);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ActionLib.RobotLift             lift            = new ActionLib.RobotLift(hardwareMap);
        ActionLib.RobotIntake      intakeSlide     = new ActionLib.RobotIntake(hardwareMap);
//        ActionLib.RobotIntakeRotator    intakeRotator   = new ActionLib.RobotIntakeRotator(hardwareMap);
        ActionLib.RobotIntakeClaw       intakeClaw      = new ActionLib.RobotIntakeClaw(hardwareMap);
        ActionLib.RobotIntakeTilter     intakeTilter    =new ActionLib.RobotIntakeTilter(hardwareMap);

        //Init robot position
        intakeClaw.clawClose();
        intakeSlide.actionIntakeUp();
        intakeTilter.tilterDown();

        TrajectoryActionBuilder rightPathToSub = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(-2, -32.5), WEST); //spline out to the sub and scoring first spec
        Action trajectoryActionToSub = rightPathToSub.build();

        initialPose = new Pose2d(-2, -32.5, WEST);
        TrajectoryActionBuilder backUpFromSub = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(24, -42),NORTH_WEST); //backup from sub
        Action trajectoryActionbackUpFromSub = backUpFromSub.build();

        initialPose = new Pose2d(24, -42, NORTH_WEST);
        TrajectoryActionBuilder pathToSample1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(42, -22), NORTH_EAST); //strafe to first sample
        Action trajectoryActionToSample1 = pathToSample1.build();

        initialPose = new Pose2d(42, -22, NORTH_EAST);
        TrajectoryActionBuilder pathPushingSample1 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(48, -36), EAST); //pushing to first sample
        Action trajectoryActionpathPushingSample1 = pathPushingSample1.build();

        initialPose = new Pose2d(48, -36, EAST);
        TrajectoryActionBuilder pathPushingSample1Part2 = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(44, -65), EAST); //pushing to first sample
        Action trajectoryActionpathPushingSample1Part2 = pathPushingSample1Part2.build();

        initialPose = new Pose2d(44, -65, EAST);
        TrajectoryActionBuilder pathToSub2 = drive.actionBuilder(initialPose)
                .strafeToLinearHeading(new Vector2d(2, -32.5), WEST); //pushing to first sample
        Action trajectoryActionpathToSub2 = pathToSub2.build();




// wait section claw down
        initialPose = new Pose2d(6.25, -65, WEST);
        TrajectoryActionBuilder pathWait1 = drive.actionBuilder(initialPose)
                .waitSeconds(0.075);
        Action trajectoryActionpathpathWait1 = pathWait1.build();


        initialPose = new Pose2d(2, -32.5, EAST);
        TrajectoryActionBuilder pathWait2 = drive.actionBuilder(initialPose)
                .waitSeconds(30);
        Action trajectoryActionpathpathWait2 = pathWait1.build();

// wait section claw up


        telemetry.addData("Status", "> INIT");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
            new SequentialAction(
                    new ParallelAction(
                            intakeSlide.actionIntakeUp(),
                            lift.actionLiftSpecimen(),
                            intakeTilter.actionTilterUp(),
                            new SequentialAction(trajectoryActionpathpathWait1,trajectoryActionToSub)),
                    lift.actionliftScore(),
                    intakeClaw.actionClawOpen(),
                    new ParallelAction(trajectoryActionbackUpFromSub,lift.actionClawGrab()),
                    trajectoryActionToSample1,
                    trajectoryActionpathPushingSample1,
                    trajectoryActionpathPushingSample1Part2,
                    new ParallelAction(intakeClaw.actionClawClose(),intakeSlide.actionIntakeUp()),
                    trajectoryActionpathToSub2,
                    trajectoryActionpathpathWait2


            )
        );
    }
}
