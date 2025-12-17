package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadRunner.tuning.TuningOpModes;

@Autonomous(name = "AutoTest1", group = "Test")
public class AutoTest1 extends LinearOpMode {
    @Override
    public void runOpMode() {

        Pose2d beginPose = new Pose2d(64.45, 12, Math.PI); // 시작 위치
        Vector2d endPose = new Vector2d(64.45, 60); // 목적지 위치
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart(); // 시작 누를때 까지 기다림

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .strafeTo(endPose)
                            .build());
        }
    }
}
