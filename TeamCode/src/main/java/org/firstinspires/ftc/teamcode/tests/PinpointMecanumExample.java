package org.firstinspires.ftc.teamcode.tests;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(name = "Pinpoint Mecanum Example", group = "Autonomous")
public class PinpointMecanumExample extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // 1. 하드웨어 설정 및 MecanumDrive 객체 생성
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // 2. 로봇의 시작 위치 설정
        Pose2d startPose = new Pose2d(0, 0, 0);
        drive.setPoseEstimate(startPose);

        // 3. 이동 경로 생성
        // 첫 번째 경로: (30, 30) 좌표까지 직선으로 이동
        Trajectory trajectory1 = drive.trajectoryBuilder(startPose)
                .lineTo(new Vector2d(30, 30))
                .build();

        // 두 번째 경로: (-10, 20) 좌표까지 부드러운 곡선으로 이동
        // 이 경로는 90도 회전 *후*에 실행될 것이므로, 시작 위치를 미리 계산할 필요 없이
        // 회전이 끝난 현재 로봇의 위치에서 경로를 만들면 됩니다.
        Trajectory trajectory3 = drive.trajectoryBuilder(new Pose2d(30, 30, Math.toRadians(90)))
                .splineTo(new Vector2d(-10, 20), Math.toRadians(180))
                .build();


        waitForStart();

        if (isStopRequested()) return;

        // 4. 생성된 경로와 회전을 순서대로 실행
        telemetry.addData("Status", "경로 1 실행 중...");
        telemetry.update();
        drive.followTrajectory(trajectory1);

        // =======================================================
        // ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼ 이 부분이 수정되었습니다 ▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼
        // =======================================================
        telemetry.addData("Status", "90도 회전 실행 중...");
        telemetry.update();
        // trajectoryBuilder를 사용하지 않고 .turn() 메소드를 직접 호출합니다.
        drive.turn(Math.toRadians(90));
        // =======================================================
        // ▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲
        // =======================================================

        telemetry.addData("Status", "경로 3 실행 중...");
        telemetry.update();
        drive.followTrajectory(trajectory3);

        telemetry.addData("Status", "자율 주행 완료!");
        telemetry.update();
    }
}
