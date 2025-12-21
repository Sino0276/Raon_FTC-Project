package org.firstinspires.ftc.teamcode.commands.mech;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;

public class WaitForTagCommand extends CommandBase {
    private final VisionSubsystem vision;
    private final int targetId;

    public WaitForTagCommand(VisionSubsystem vision, int targetId) {
        this.vision = vision;
        this.targetId = targetId;
        // VisionSubsystem은 '읽기 전용'으로 쓰이므로
        // addRequirements(vision)을 굳이 하지 않아도 됨. (다른 커맨드 방해 X)
    }

    @Override
    public boolean isFinished() {
        // 태그가 보이면 true를 반환하여 커맨드 종료 -> 다음 순서로 넘어감
        return vision.isTagVisible(targetId);
    }
}
