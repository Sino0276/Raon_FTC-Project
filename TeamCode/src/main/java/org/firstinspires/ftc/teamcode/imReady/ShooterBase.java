package org.firstinspires.ftc.teamcode.imReady;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class ShooterBase extends ShooterHardware {
    public static PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(35, 0, 10, 13);
    public static double TPS = 1000;
    public static double DEFAULT_TPS = 1000;
    public static double SERVO_MAX = 0.8;
    public static double SERVO_MIN = 0;

    // 물리적 제원 (보정)
    public static double FLYWHEEL_RADIUS = 1.889765; // 바퀴 반지름
    public static double SHOOTER_EFFICENCY = 0.59; // 효율계수 (0.0 ~ 1.0) 1은 슬립 X // 튜닝 할 것
    public static double TICK_PER_REV = 28; // 모터 해상도 PPR
    public static double GEAR_RATIO = 1; // 기어비

    // 포물선 운동 상수
    private double x = 0; // 수평 거리 (in)
    private final double delta_h = 30.098425; // 수직 높이 차이 (in)
    private final double theta = 0.872665; // 기본 발사 각도 (rad)
    private final double g = 386.1; // 중력 가속도 (in/s^2)

    public boolean isShooterSpin = false,
            isServoSpin = false;

    private CameraBase camera;

    public ShooterBase(HardwareMap hardwareMap) {
        super(hardwareMap);
        camera = CameraBase.getInstance(hardwareMap);
    }

    public void update(Gamepad gamepad) {
        shooterLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, SHOOTER_PID);
        shooterRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, SHOOTER_PID);

        if (gamepad.bWasPressed())
            gamepad.rumble(1000);

        if (gamepad.xWasPressed()) {

            isShooterSpin = !isShooterSpin;
            if (isShooterSpin) {

                if (camera.isTagVisible(20)) {
                    AprilTagDetection detection = camera.getDetectionById(20);
                    // 거리 기반 TPS 조절 (예시 값, 실제로는 거리 계산 필요)
                    x = detection.ftcPose.y;

                    // 발사 불가능한 각도인지 (Shadow Zone)체크
                    // 즉, 발사 각도로는 해당 높이(y)에 도달할 수 없음을 의미
                    double discriminat = (x * Math.tan(theta)) - delta_h;
                    if (discriminat > 0) {
                        // 초기 선속도(v0)
                        double v0 = Math.sqrt((g * x * x) / (2 * Math.pow(Math.cos(theta), 2) * discriminat));
                        // 선속도(v0) -> TPS 변환
                        double requiredTPS = calculateTPSFromVelocity(v0);
                        setVelocity(requiredTPS);
                    } else {
                        setVelocity(DEFAULT_TPS);
                    }

                } else {
                    setVelocity(DEFAULT_TPS);
                }

            } else {
                setPower(0);
            } // 모터 꺼짐
        }

        // 슈터 속도 조절
        // if (gamepad.dpadUpWasPressed()) { addTPS(10);}
        // else if (gamepad.dpadDownWasPressed()) { addTPS(-10);}
        // else if (gamepad.dpadLeftWasPressed()) { addTPS(1);}
        // else if (gamepad.dpadRightWasPressed()){ addTPS(-1);}
        if (gamepad.dpadUpWasPressed()) {
            SHOOTER_EFFICENCY += 0.01;
        } else if (gamepad.dpadDownWasPressed()) {
            SHOOTER_EFFICENCY -= 0.01;
        } else if (gamepad.dpadLeftWasPressed()) {
            SHOOTER_EFFICENCY += 0.001;
        } else if (gamepad.dpadRightWasPressed()) {
            SHOOTER_EFFICENCY -= 0.001;
        }

        // 서보 회전
        if (gamepad.yWasPressed()) {

            isServoSpin = !isServoSpin;
            // if (servo.getPower() == SERVO_MIN) { ㄱㄴ?
            if (isServoSpin) {
                servoSpin(SERVO_MAX); // 여기
            } else {
                servoSpin(SERVO_MIN); // 여기
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
        if (TPS > 2800) {
            this.TPS = 2800;
        } else if (TPS < 0) {
            this.TPS = 0;
        } else {
            this.TPS = TPS;
        }
    }

    public void addTPS(double amount) {
        setTPS(TPS + amount);
    }

    /**
     * 이론적 선속도(v0)를 목표 TPS로 변환
     * 공식: TPS (v0 / (r * efficiency)) * (ticks_per_rev) / (2 * pi)) * gear_ratio
     * 
     * @param v0 필요한 초기 발사속도 (in/s)
     * @return Motor Ticks Per Second (TPS)
     */
    private double calculateTPSFromVelocity(double v0) {
        // 실제 휠이 내야하는 접선의 속도 (슬립 등 비효율 고려)
        double wheelTangentialVelocity = v0 / SHOOTER_EFFICENCY;
        // 필요한 각속도 (rad/s) = v / r
        double angularVelocity = wheelTangentialVelocity / FLYWHEEL_RADIUS;
        // Rad/s -> TPS변환
        // 1회전 = 2pi rad = TICKS_PER_REV
        double tps = angularVelocity * (TICK_PER_REV / (2 * Math.PI)) * GEAR_RATIO;

        return tps;
    }

    // servo

    public void servoSpin(double power) {
        servo.setPower(power);
        // servo.getController().getServoPosition();
    }
}
