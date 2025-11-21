package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Config
public class CameraBase extends CameraHardware {
    // 싱글톤 인스턴스
    private static CameraBase instance = null;
    public static double BEARING_OFFSET = 0.17; // 필요시 보정 각도 설정 rad

    private final int APRILTAG_FIRST_ID = 20;
    public List<AprilTagDetection> currentDetections;

    private CameraBase(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void update() {
        updateDetections();

    }

    // 싱글톤 패턴을 사용하여 인스턴스 반환
    public static CameraBase getInstance(HardwareMap hardwareMap) {
        if (instance == null) {
            instance = new CameraBase(hardwareMap);
        }
        return instance;
    }

    public void enableCameraStreaming(int maxFps) {
        FtcDashboard.getInstance().startCameraStream(visionPortal, maxFps);
    }

    public void updateDetections() {
        currentDetections = aprilTag.getDetections();
    }

    public AprilTagDetection getDetectionById(int id) {
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == id) {
                return detection;
            }
        }
        return null; // 해당 ID의 태그가 감지되지 않음
    }

    public boolean isTagVisible(int id) {
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == id) {
                return true;
            }
        }
        return false;
    }

    public Position getRobotPoseFromTag(int id) {
        AprilTagDetection detection = getDetectionById(id);
        if (detection != null) {
            return detection.robotPose.getPosition();
        }
        return null; // 해당 ID의 태그가 감지되지 않음
    }

    public double getCorrectedBearing(AprilTagDetection detection, double camXOffset, double camYOffset) {
        // 1. 기존 데이터 가져오기 (Degree -> Radian 변환 필수)
        double range = detection.ftcPose.range;
        double bearingRad = Math.toRadians(detection.ftcPose.bearing);

        // 2. 카메라 기준의 태그 좌표 계산 (전방이 X, 좌측이 Y라고 가정)
        // 주의: 삼각함수 축은 로봇의 좌표계 설정에 따라 cos/sin이 달라질 수 있습니다.
        // 일반적인 수학 좌표계(X가 가로)와 로봇 좌표계(X가 전방) 차이를 주의하세요.
        // 여기서는 FTC 일반적인 Pose 기준(X: 전방, Y: 좌측)으로 작성합니다.
        double tagXCam = range * Math.cos(bearingRad);
        double tagYCam = range * Math.sin(bearingRad);

        // 3. 오프셋 적용 (로봇 중심 좌표로 변환)
        double targetX = tagXCam + camXOffset;
        double targetY = tagYCam + camYOffset;

        // 4. 새로운 각도(Bearing) 계산 (Radian -> Degree 복구)
        double correctedBearingRad = Math.atan2(targetY, targetX);

        return correctedBearingRad;
    }

    public void closeCamera() {
        dashboard.stopCameraStream();
        visionPortal.close();
        instance = null;
    }

}
