package org.firstinspires.ftc.teamcode.hardwares;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@Config
public class CameraBase extends CameraHardware {
    // 싱글톤 인스턴스
    private static CameraBase instance = null;
    public static double BEARING_OFFSET = 0.1; // 필요시 보정 각도 설정 rad

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

    public double getDistanceByTag(AprilTagDetection detection) {
        if (detection != null) {
            return detection.ftcPose.y;
        }
        return 0.0; // AprilTag 감지X
    }
    public double getDistanceById(int id) {
        AprilTagDetection detection = getDetectionById(id);
        return getDistanceByTag(detection);
    }

    public void closeCamera() {
        dashboard.stopCameraStream();
        visionPortal.close();
        instance = null;
    }

}
