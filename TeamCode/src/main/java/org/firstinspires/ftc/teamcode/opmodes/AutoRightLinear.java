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
@Autonomous(name = "AUTO_RIGHT_LINEAR", group = "Autonomous")
public class AutoRightLinear extends LinearOpMode {

    double SOUTH = Math.PI;
    double EAST = 3 * Math.PI / 2;
    double NORTH = 0;
    double WEST = Math.PI / 2;

    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(16.5, -65, WEST);
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ActionLib.RobotLift             lift            = new ActionLib.RobotLift(hardwareMap);
//        ActionLib.RobotIntakeSlide      intakeSlide     = new ActionLib.RobotIntakeSlide(hardwareMap);
//        ActionLib.RobotIntakeRotator    intakeRotator   = new ActionLib.RobotIntakeRotator(hardwareMap);
        ActionLib.RobotIntakeClaw       intakeClaw      = new ActionLib.RobotIntakeClaw(hardwareMap);

        //Init robot position
        intakeClaw.clawClose();

        TrajectoryActionBuilder rightPathToSub = drive.actionBuilder(initialPose)
                .splineToConstantHeading(new Vector2d(6, -32.75), WEST); //spline out to the sub

        TrajectoryActionBuilder rightPathDropSamples = drive.actionBuilder(initialPose)
                .waitSeconds(0.5)
                .splineToConstantHeading(new Vector2d(24,-48), WEST) //backs up from sub
                .waitSeconds(0.01)
                .splineToConstantHeading(new Vector2d(36,-24 ), WEST)
                .turnTo(NORTH) //turn 90 degrease
                .lineToXConstantHeading(46)//captures first block
                .turnTo(EAST)
                .splineToConstantHeading(new Vector2d(48, -60), EAST) // pushes the the first block
                .lineToY(-12)
                .strafeToConstantHeading(new Vector2d(54, -12)) //strafe aiming for the second block
                .waitSeconds(0.01)
                .strafeToConstantHeading(new Vector2d(54,-60)) //pushes the second block into the zone
                .lineToY(-12)
                .strafeToConstantHeading(new Vector2d(61.0, -12)) //strafe aiming for the third block
                .waitSeconds(0.01)
                .turnTo(EAST)
                .strafeToConstantHeading(new Vector2d(60,-60)); //pushes the third block into the zone

        TrajectoryActionBuilder wait = drive.actionBuilder(initialPose)
                .waitSeconds(0.5);

        Action trajectoryActionToSub = rightPathToSub.build();
        Action trajectoryActionDropSamples = rightPathDropSamples.build();
        Action trajectoryWait = rightPathDropSamples.build();

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
                lift.actionLiftDown(),
                new ParallelAction(
                    trajectoryActionDropSamples,
                    new SequentialAction( lift.actionLiftDown() )
                )
            )
        );
    }
}
