package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class ShooterBase extends ShooterHardware {
    public static PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(100, 0.3, 0.5, 11);
    public static int LEFT_TPS = 1000;
    public static int RIGHT_TPS = 1000;
    public static double SERVO_MAX = 0.8;
    public static double SERVO_MIN = 0;

    public boolean  isShooterSpin = false,
                    isServoSpin = false;

    private CameraBase camera;

    public ShooterBase(HardwareMap hardwareMap) {
        super(hardwareMap);
        camera = CameraBase.getInstance(hardwareMap);
    }

    public void Update(Gamepad gamepad) {
//        shooterLeft.setPIDFCoefficients(DcMotorEx.RunMode.VELOCITY, SHOOTER_PID);
//        shooterRight.setPIDFCoefficients(DcMotorEx.RunMode.VELOCITY, SHOOTER_PID);

        if (gamepad.xWasPressed()) {

            isShooterSpin = !isShooterSpin;
            // if
            if (isShooterSpin) {
                if (camera.isTagVisible(20)) {
                    AprilTagDetection detection = camera.getDetectionById(20);
                    //   거리 기반 TPS 조절 (예시 값, 실제로는 거리 계산 필요)

                } else {
                    setVelocity(LEFT_TPS, RIGHT_TPS);
                }

            } else {
                setPower(0, 0);
            }
        }
        // 슈터 속도 조절
        else if (gamepad.dpadUpWasPressed())   { addTPS(10);}
        else if (gamepad.dpadDownWasPressed()) { addTPS(-10);}
        else if (gamepad.dpadLeftWasPressed()) { addTPS(1);}
        else if (gamepad.dpadRightWasPressed()){ addTPS(-1);}

        // 서보 회전
        if (gamepad.yWasPressed()) {

            isServoSpin = !isServoSpin;
            // if (servo.getPower() == SERVO_MIN) {  ㄱㄴ?
            if (isServoSpin) {
                servoSpin(SERVO_MAX);     // 여기
            } else {
                servoSpin(SERVO_MIN);     // 여기
            }
        }
    }

    public void setVelocity(int left_TPS, int right_TPS) {
        shooterLeft.setVelocity(left_TPS);
        shooterRight.setVelocity(right_TPS);
    }

    public void setPower(double left_power, int right_power) {
        shooterLeft.setPower(left_power);
        shooterRight.setPower(right_power);
    }

    public void setTPS(int left_TPS, int right_TPS) {
        setLeftTPS(left_TPS);
        setRightTPS(right_TPS);
    }
    public void setLeftTPS(int left_TPS) {
        if (left_TPS > 2800)    { this.LEFT_TPS = 2800; }
        else if (left_TPS < 0)  { this.LEFT_TPS = 0; }
        else                    { this.LEFT_TPS = left_TPS; }
    }
    public void setRightTPS(int right_TPS) {
        if (right_TPS > 2800)   { this.RIGHT_TPS = 2800; }
        else if (right_TPS < 0) { this.RIGHT_TPS = 0; }
        else                    { this.RIGHT_TPS = right_TPS; }
    }

    public void addTPS(int amount) {
        setTPS(LEFT_TPS + amount,
                RIGHT_TPS + amount);
    }

    public void addLeftTPS(int amount) {
        setTPS(LEFT_TPS + amount,
                RIGHT_TPS - amount);
    }
    public void addRightTPS(int amount) {
        setTPS(LEFT_TPS - amount,
                RIGHT_TPS + amount);
    }

    // servo

    public void servoSpin(double power) {
        servo.setPower(power);
    }
}
