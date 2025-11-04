package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.imReady.ShooterBase;
import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;

@Autonomous(name = "AutoTest4")public class AutoTest4 extends LinearOpMode {
    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(64.45, 12, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        ShooterBase shoot = new ShooterBase(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 2;

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .afterTime(0, () -> shoot.setVelocity(shoot.LEFT_TPS, shoot.RIGHT_TPS))
                .afterTime(1, () -> shoot.servoSpin(1))
                .afterTime(8, () -> shoot.servoSpin(0))
                .turn(Math.toRadians(25));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(64.45, 12, Math.toRadians(205)))
//                .waitSeconds(1)
                .strafeToSplineHeading(new Vector2d(40.45, 32), Math.toRadians(180));


        Action trajectoryActionCloseOut = tab2.endTrajectory().fresh()
                .strafeTo(new Vector2d(64.45, 12))
                .build();

        // actions that need to happen on init; for instance, a claw tightening.
//        Actions.runBlocking(claw.closeClaw());


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(
                new SequentialAction(
                        tab1.build(),
                        tab2.build(),
                        trajectoryActionCloseOut
                )
        );

    }
}
