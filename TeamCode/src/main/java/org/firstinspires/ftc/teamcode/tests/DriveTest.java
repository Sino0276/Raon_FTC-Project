package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.*;

import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;
@Disabled
@TeleOp(name = "DriveTest", group = "Test")
public class DriveTest extends LinearOpMode {

    // P제어 게인 (FTC Dashboard에서 조정 가능하도록 static으로 선언)
    public static double HEADING_P_GAIN = 2.0;

    // 목표 방향 (라디안)
    private double targetHeading = 0.0;
    private boolean headingLockEnabled = false;

    @Override
    public void runOpMode() throws InterruptedException {
        // MecanumDrive 초기화 (시작 위치 0, 0, 0)
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        telemetry.addLine("로봇 준비 완료");
        telemetry.addLine("왼쪽 스틱: 이동");
        telemetry.addLine("오른쪽 스틱 X: 회전");
        telemetry.addLine("Dpad: 방향 고정 (↑:앞 ↓:뒤 ←:좌 →:우)");
        telemetry.addLine("Right Bumper: 느린 모드");
        telemetry.addLine("Left Bumper: 방향 고정 해제");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // 시작 시 현재 방향을 목표 방향으로 설정
        targetHeading = drive.localizer.getPose().heading.log();

        while (opModeIsActive()) {
            // 게임패드 입력 받기
            double y = -gamepad1.left_stick_y;  // 전진/후진 (Y축 반전)
            double x = gamepad1.left_stick_x;   // 좌/우 스트레이프
            double rx = gamepad1.right_stick_x; // 회전

            // Dpad로 목표 방향 설정 (필드 좌표 기준)
            if (gamepad1.dpad_up) {
                targetHeading = Math.toRadians(90);  // 앞 (필드 기준 위쪽)
                headingLockEnabled = true;
            } else if (gamepad1.dpad_down) {
                targetHeading = Math.toRadians(-90); // 뒤 (필드 기준 아래쪽)
                headingLockEnabled = true;
            } else if (gamepad1.dpad_left) {
                targetHeading = Math.toRadians(180); // 좌 (필드 기준 왼쪽)
                headingLockEnabled = true;
            } else if (gamepad1.dpad_right) {
                targetHeading = Math.toRadians(0);   // 우 (필드 기준 오른쪽)
                headingLockEnabled = true;
            }

            // Left Bumper로 방향 고정 해제
            if (gamepad1.left_bumper) {
                headingLockEnabled = false;
            }

            // 느린 모드 (Right Bumper 누르면 30% 속도)
            double speedMultiplier = gamepad1.right_bumper ? 0.3 : 1.0;

            // 현재 로봇 위치 및 방향
            Pose2d currentPose = drive.localizer.getPose();
            double currentHeading = currentPose.heading.log();

            // 조이스틱 데드존 체크
            boolean isMoving = Math.abs(y) > 0.1 || Math.abs(x) > 0.1;
            boolean isRotating = Math.abs(rx) > 0.1;

            // 회전 제어 계산
            double rotationPower;

            if (headingLockEnabled && !isRotating && !isMoving) {
                // 방향 고정 모드: 조이스틱을 완전히 놨을 때만 P제어 작동
                double headingError = normalizeAngle(targetHeading - currentHeading);
                rotationPower = headingError * HEADING_P_GAIN;

                // 회전 속도 제한
                rotationPower = Math.max(-1.0, Math.min(1.0, rotationPower));
            } else {
                // 수동 회전 모드 또는 이동 중
                rotationPower = rx;

                // 수동으로 회전 중일 때는 현재 방향을 새로운 목표로 설정
                if (isRotating) {
                    targetHeading = currentHeading;
                    headingLockEnabled = true;
                }
            }

            // 필드 중심 제어 (Field-Centric Drive)
            // 로봇의 현재 방향을 고려하여 조이스틱 입력을 필드 좌표로 변환
            double sin = Math.sin(-currentHeading);
            double cos = Math.cos(-currentHeading);
            double rotatedY = y * cos - x * sin;
            double rotatedX = y * sin + x * cos;
            Vector2d input = new Vector2d(rotatedY, rotatedX);

            // 로봇에 속도 명령 전송
            drive.setDrivePowers(new PoseVelocity2d(
                    input.times(speedMultiplier),
                    rotationPower * speedMultiplier
            ));

            // 오도메트리 업데이트
            drive.updatePoseEstimate();

            // 텔레메트리 출력
            telemetry.addData("모드", gamepad1.right_bumper ? "느림" : "보통");
            telemetry.addData("방향 고정", headingLockEnabled ? "활성화" : "비활성화");
            telemetry.addData("", "");
            telemetry.addData("현재 방향 (도)", "%.1f", Math.toDegrees(currentHeading));
            telemetry.addData("목표 방향 (도)", "%.1f", Math.toDegrees(targetHeading));
            telemetry.addData("방향 오차 (도)", "%.1f", Math.toDegrees(normalizeAngle(targetHeading - currentHeading)));
            telemetry.addData("", "");
            telemetry.addData("X 위치", "%.2f", currentPose.position.x);
            telemetry.addData("Y 위치", "%.2f", currentPose.position.y);
            telemetry.addData("회전 출력", "%.2f", rotationPower);
            telemetry.update();

        }
    }

    /**
     * 각도를 -π ~ π 범위로 정규화
     */
    private double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }
}
