package org.firstinspires.ftc.teamcode.hardwares;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Config
public class ShooterBase extends ShooterHardware {
    public static PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(35, 0, 10, 13);
    public static double TPS = 1000;
    public static double DEFAULT_TPS = 980;
    public static double SERVO_MAX = 0.8;
    public static double SERVO_MIN = 0;
    public static double SERVO_WAIT_TIME = 1100; // ms

    private ElapsedTime shootTimer = new ElapsedTime();
    private boolean isShooting = false;

    // 물리적 제원 (보정)
    public static double FLYWHEEL_RADIUS = 1.889765; // 바퀴 반지름
    public static double SHOOTER_EFFICENCY = 0.58; // 효율계수 (0.0 ~ 1.0) 1은 슬립 X // 튜닝 할 것
    public static double TICK_PER_REV = 28; // 모터 해상도 PPR
    public static double GEAR_RATIO = 1; // 기어비

    // 포물선 운동 상수
    private double x = 0; // 수평 거리 (in)
    private final double delta_h = 30.098425; // 수직 높이 차이 (in)
    private final double theta = 0.872665; // 기본 발사 각도 (rad)
    private final double g = 386.1; // 중력 가속도 (in/s^2)

    public boolean isShooterSpin = false;

    private CameraBase camera;

    public ShooterBase(HardwareMap hardwareMap) {
        super(hardwareMap);
        camera = CameraBase.getInstance(hardwareMap);
        setServoPosition(SERVO_MIN);
    }

    public void update(Gamepad gamepad) {
        shooterLeft.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, SHOOTER_PID);
        shooterRight.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, SHOOTER_PID);

        if (gamepad.xWasPressed()) {

            isShooterSpin = !isShooterSpin;
            if (isShooterSpin) {

                if      (camera.isTagVisible(20)) { activeShooterTps(camera.getDetectionById(20)); }
                else if (camera.isTagVisible(24)) { activeShooterTps(camera.getDetectionById(24)); }
                else                                 { setVelocity(DEFAULT_TPS); }

            } else { setPower(0); } // 모터 꺼짐
        }

        // 슈터 효율 조절
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
        if (gamepad.y) {
            pushBall();
        }
        updateCycle();

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

    public void activeShooterTps(AprilTagDetection detection) {
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

    public void setServoPosition(double position) {
        servo.setPosition(position);
        // servo.getController().getServoPosition();
    }

    public void pushBall() {
        if (!isShooting) {
            isShooting = true;
            shootTimer.reset();
            setServoPosition(SERVO_MAX);
        }
    }

    /**
     * 루프 방식이 아닌 자율주행의 순차적 진행 방식에선
     * while문을 묶어서 사용하자
     * <p>
     * pushBall()과 updateCycle() 메소드는 한 묶음이며,
     * updateCycle() 메소드는 반복되며 조건문을 처리하지 않으면
     * 서보가 원위치로 돌아오지 않는다.
     *
     * @return 서보가 제자리로 돌아오면 false를 반환 / 아직 돌아오지 않았다면 true를 반환
     */
    public boolean updateCycle() {
        boolean returnValue = true;

        if (isShooting) {
            if (shootTimer.milliseconds() > SERVO_WAIT_TIME) {
                setServoPosition(SERVO_MIN);
                returnValue = true;
            }

            if (shootTimer.milliseconds() > SERVO_WAIT_TIME * 2) {
                isShooting = false;
                returnValue = false;
            }
        }

        return returnValue;
    }
}
