package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class ShooterBase extends ShooterHardware {
    public static PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(35, 0, 10, 13);
    public static double TPS = 1000;
    public static double DEFAULT_TPS = 1000;
    public static double K_DISTANCE = 4.15; // 거리 기반 속도 조절 계수
    public static double SERVO_MAX = 0.8;
    public static double SERVO_MIN = 0;

    private double x = 0; // 수평 거리 (in)
    private final double y = 28.7402;          // 수직 높이 차이 (in)
    private final double theta = 0.85956;     // 기본 발사 각도 (rad)
    private final double g = 386.1;           // 중력 가속도 (in/s^2)

    public boolean  isShooterSpin = false,
                    isServoSpin = false;

    private CameraBase camera;

    public ShooterBase(HardwareMap hardwareMap) {
        super(hardwareMap);
        camera = CameraBase.getInstance(hardwareMap);
    }

    public void Update(Gamepad gamepad) {
        shooterLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, SHOOTER_PID);
        shooterRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, SHOOTER_PID);

        if (gamepad.bWasPressed()) gamepad.rumble(1000);

        if (gamepad.xWasPressed()) {
            isShooterSpin = !isShooterSpin;
            if (isShooterSpin) {
                // 오른쪽, 왼쪽 모터의 속력이 비슷한가? && 두 모터속도의 평균이 TPS와 근접한가?
//                if (Math.abs(shooterLeft.getVelocity() - shooterRight.getVelocity()) < 10 && ((shooterLeft.getVelocity() + shooterRight.getVelocity()) / 2) - TPS < 5) { gamepad.rumble(1000); }


                if (camera.isTagVisible(20)) {
                    AprilTagDetection detection = camera.getDetectionById(20);
                    //   거리 기반 TPS 조절 (예시 값, 실제로는 거리 계산 필요)
                    x = detection.ftcPose.y;
//                    if (!((x * Math.tan(theta)) - y <= 0)) {
                        // (x * tan(theta)) <= y 라는 의미
                        // 즉, 발사 각도로는 해당 높이(y)에 도달할 수 없음을 의미
                        double v0 = Math.sqrt((g * x * x) / (2 * Math.pow(Math.cos(theta), 2) * (x * Math.tan(theta) - y)));
                        setVelocity(K_DISTANCE * v0);
//                    }

                } else { setVelocity(DEFAULT_TPS); }


            } else { setPower(0); }
        }

        // 슈터 속도 조절
        if      (gamepad.dpadUpWasPressed())   { addTPS(10);}
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

    public void setVelocity(double TPS) {
        setTPS(TPS);
        shooterLeft.setVelocity(this.TPS);
        shooterRight.setVelocity(this.TPS);
    }
    public void addVelocity(double amount) {
        setVelocity(this.TPS + amount);
    }

    public void setPower(double power) {
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
    }

    private void setTPS(double TPS) {
        if (TPS > 2800)    { this.TPS = 2800; }
        else if (TPS < 0)  { this.TPS = 0; }
        else               { this.TPS = TPS; }
    }
    public void addTPS(double amount) {
        setTPS(TPS + amount);
    }

    // servo

    public void servoSpin(double power) {
        servo.setPower(power);
//        servo.getController().getServoPosition();
    }
}
