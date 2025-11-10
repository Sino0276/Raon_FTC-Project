package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.Comparator;
import java.util.List;

public class CameraBase extends CameraHardware {
    private final int APRILTAG_FIRST_ID = 20;
    public List<AprilTagDetection> currentDetections;

    public CameraBase(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public void Update() {
        updateDetections();

    }

    public void enableCameraStreaming(int maxFps) {
        FtcDashboard.getInstance().startCameraStream(visionPortal, maxFps);
    }

    public void updateDetections() {
        currentDetections = aprilTag.getDetections();
        currentDetections.sort(Comparator.comparingInt(detection -> detection.id));
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

}
