package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "AutoTest")
public class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() {

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // 2. 로봇의 시작 위치 설정
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        // 첫 번째 경로: (30, 30) 좌표까지 직선으로 이동
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(30, 30))
                .build();

        waitForStart();
        if (isStopRequested()) return;

        drive.followTrajectory(trajectory1);
    }
}
