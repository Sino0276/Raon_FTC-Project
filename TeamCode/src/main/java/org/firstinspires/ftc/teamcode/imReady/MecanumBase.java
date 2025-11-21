package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.roadRunner.MecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class MecanumBase {
    public static double KP = 2;
    public static double KD = 0.2;

    private double lastHeading, currentHeading, targetHeading, postHeading;
    private ElapsedTime time;
    private double lastTime = 0;
    private double speedMultiplier;
    private boolean isFieldCentric = true; // 필드 중심 제어 (Field-Centric Drive)

    private MecanumDrive drive;
    private CameraBase camera;

    public MecanumBase(HardwareMap hardwareMap, Pose2d pose2d) {
        camera = CameraBase.getInstance(hardwareMap);
        drive = new MecanumDrive(hardwareMap, pose2d);

        lastHeading = currentHeading = targetHeading = postHeading = drive.localizer.getPose().heading.log();
        time = new ElapsedTime();
    }

    public void update(Gamepad gamepad) {
        double x = -gamepad.left_stick_x;
        double y = -gamepad.left_stick_y;
        double rx = -gamepad.right_stick_x;

        if (gamepad.aWasPressed()) {
            isFieldCentric = !isFieldCentric;
        }

        currentHeading = drive.localizer.getPose().heading.log();
        drive.updatePoseEstimate();

        if (gamepad.bWasPressed()) {
            // 카메라가 태그 20을 인식 중일 때만 방향 갱신
            if (camera.isTagVisible(20)) {
                turnToAprilTag(camera.getDetectionById(20));
            }
            // 카메라가 태그 24를 인식 중일 때만 방향 갱신
            else if (camera.isTagVisible(24)) {
                turnToAprilTag(camera.getDetectionById(24));
            }

//            AprilTagDetection detection = camera.getDetectionById(20);
//            targetHeading = currentHeading - camera.getDetectionById(20).ftcPose.bearing + camera.BEARING_OFFSET;;

        }

        if (gamepad.dpad_up) {
            targetHeading = postHeading;
        } else if (gamepad.dpad_left) {
            targetHeading = postHeading + Math.toRadians(90);
        } else if (gamepad.dpad_down) {
            targetHeading = postHeading + Math.toRadians(180);
        } else if (gamepad.dpad_right) {
            targetHeading = postHeading + Math.toRadians(270);
        } else {
            speedMultiplier = (gamepad.right_bumper || gamepad.left_bumper) ? 0.3 : 0.7;
            move(x, y, rx);
        }
    }

    /**
     * MecanumDrive 인스턴스를 반환
     * @return MecanumDrive
     */
    public MecanumDrive getRoadRunnerDrive() {
        return drive;
    }

    public void move(double x, double y, double rx) {
        double xPower, yPower, rotationPower;

        double dt = time.seconds() - lastTime;
        lastTime = time.seconds();
        double derivative = 0;
        if (dt > 0) {
            derivative = normalizeAngle(currentHeading - lastHeading) / dt;
        }
        lastHeading = currentHeading;
        if (rx == 0) {
            // rx 조이스틱을 완전히 놨을 때만 PD제어 작동
            double headingError = normalizeAngle(targetHeading - currentHeading);
            rotationPower = (KP * headingError) - (KD * derivative);

            // 회전 속도 제한
            rotationPower = Range.clip(rotationPower, -1.0, 1.0);
        } else {
            // 수동 회전 모드 또는 이동 중
            rotationPower = rx;

            // 수동으로 회전 중일 때는 현재 방향을 새로운 목표로 설정
            targetHeading = currentHeading;
        }

        if (isFieldCentric) {
            // 필드 중심 제어 (Field-Centric Drive)
            // 로봇의 현재 방향을 고려하여 조이스틱 입력을 필드 좌표로 변환
            double sin = Math.sin(-currentHeading);
            double cos = Math.cos(-currentHeading);
            yPower = y * cos - x * sin;
            xPower = y * sin + x * cos;
        } else {
            // 로봇 중심 제어 (Robot-Centric Drive)
            yPower = y;
            xPower = x;
        }

        Vector2d input = new Vector2d(yPower, xPower);

        // 로봇에 속도 명령 전송
        drive.setDrivePowers(new PoseVelocity2d(
                input.times(speedMultiplier),
                rotationPower * speedMultiplier));
    }

    private double normalizeAngle(double angle) {
        while (angle > Math.PI)
            angle -= 2 * Math.PI;
        while (angle <= -Math.PI)
            angle += 2 * Math.PI;
        return angle;
    }

    /**
     * AprilTag null Check하기
     * @param detection
     */
    public void turnToAprilTag(AprilTagDetection detection) {
        Action TurnAction = drive.actionBuilder(drive.localizer.getPose())
                .turn(-detection.ftcPose.bearing + CameraBase.BEARING_OFFSET)
                .build();

        Actions.runBlocking(TurnAction);
        drive.updatePoseEstimate();
        currentHeading = drive.localizer.getPose().heading.log();
        targetHeading = currentHeading;

    }
}
