package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class PusherBase extends PusherHardware {
    public static double SERVO_MAX = 0.8;
    public static double SERVO_MIN = 0;

    public enum State {
        ACTIVE,     // 회전 중
        UNACTIVE    // !회전 중
    }
    private State currentState = State.UNACTIVE;
    public PusherBase(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    public State getState() { return currentState; }
    public void setState(State state) { currentState = state; }

    public void moveMaxPosition() { servo.setPosition(SERVO_MAX); }
    public void moveMinPosition() { servo.setPosition(SERVO_MIN); }
}
