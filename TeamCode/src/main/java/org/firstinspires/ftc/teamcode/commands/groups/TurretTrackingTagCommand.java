package org.firstinspires.ftc.teamcode.commands.groups;

import static org.firstinspires.ftc.teamcode.Utils.AprilTagPosition.APRILTAG_POS;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class TurretTrackingTagCommand extends CommandBase {
    private final TurretSubsystem turret;
    private final VisionSubsystem vision;
    private final Follower follower;
    private final int tagID;
    private AprilTagDetection aprilTag;
    private double angle;

    public TurretTrackingTagCommand(TurretSubsystem turret, VisionSubsystem vision, Follower follower, int tagID) {
        this.turret = turret;
        this.vision = vision;
        this.tagID = tagID;
        this.follower = follower;


        addRequirements(turret, vision);
    }

    // 커맨드 실행 중
    @Override
    public void execute() {
        double targetAngle;

        // 1. 카메라에 태그가 보이는지 확인
        if (vision.isTagVisible(tagID)) {
            // [Vision] 현재 각도 + 오차(Bearing) = 목표 절대 각도
            // 예: 현재 30도, 오차 +10도(왼쪽) -> 40도를 바라봐라
            targetAngle = turret.getAngle() + vision.getAngle(tagID);
        }
        // 2. 태그가 안 보이면 오도메트리 사용
        else {
            // 태그 정보가 없으면 현재 각도 유지 (혹은 0도 정렬)
            if (!APRILTAG_POS.containsKey(tagID)) {
                targetAngle = turret.getAngle();
            } else {
                Pose tagPose = APRILTAG_POS.get(tagID);
                Pose robotPose = follower.getPose();

                // 1. 태그 방향 벡터 각도 (필드 절대 각도)
                double deltaX = tagPose.getX() - robotPose.getX();
                double deltaY = tagPose.getY() - robotPose.getY();
                double fieldHeading = Math.toDegrees(Math.atan2(deltaY, deltaX));

                // 2. 로봇 헤딩 보정 (필드 각도 - 로봇 헤딩 = 터렛 목표 각도)
                double robotHeading = Math.toDegrees(robotPose.getHeading());

                // 이것이 바로 터렛이 바라봐야 할 '목표 각도'입니다.
                targetAngle = fieldHeading - robotHeading;
            }
        }

        // 최종 계산된 '목표 각도'로 이동 (TurnToAngle 내부에서 정규화/Safe Limit 처리됨)
        turret.turnToAngle(targetAngle);
    }

    // 커맨드 종료 시
    @Override
    public void end(boolean interrupted) {
        turret.stop();
    }

    // 커맨드 종료 조건
    @Override
    public boolean isFinished() {
        // 종료 조건이 없음
        // 강제 종료 될때 까지 계속 반복
        return false;
    }

}
