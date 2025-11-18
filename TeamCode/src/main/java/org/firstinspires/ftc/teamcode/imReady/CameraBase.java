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
    // PID 및 기본 설정
    public static PIDFCoefficients SHOOTER_PID = new PIDFCoefficients(35, 0, 10, 13);
    public static double DEFAULT_TPS = 1000;
    public static double SERVO_MAX = 0.8;
    public static double SERVO_MIN = 0;

    // 물리적 제원 (Physical Constants) - 보정 필요
    public static double FLYWHEEL_RADIUS = 1.476; // (in) 예: 75mm 휠 반지름 ~= 1.476 inch
    public static double SHOOTER_EFFICIENCY = 0.85; // 효율 계수 (0.0 ~ 1.0), 1.0은 미끄러짐 없음
    public static double TICKS_PER_REV = 28.0; // 모터 사양 (예: HD Hex Motor 등)
    public static double GEAR_RATIO = 1.0; // 모터:휠 기어비 (1:1이면 1.0)

    // 포물선 운동 상수
    private double x = 0; // 수평 거리 (in)
    private final double y = 28.7402;          // 수직 높이 차이 (in)
    private final double theta = 0.85956;     // 기본 발사 각도 (rad)
    private final double g = 386.1;           // 중력 가속도 (in/s^2)

    // 상태 변수
    public boolean  isShooterSpin = false,
                    isServoSpin = false;
    private double currentTargetTPS = 0;

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
        }

        // 슈터 로직
        if (isShooterSpin) {
            if (camera.isTagVisible(20)) {
                AprilTagDetection detection = camera.getDetectionById(20);
                
                // 거리 측정 (카메라 좌표계에 따라 y가 거리일 수 있음, 확인 필요)
                x = detection.ftcPose.y; 

                // 발사 불가능한 각도인지(Shadow zone) 체크
                double discriminant = x * Math.tan(theta) - y;
                
                if (discriminant > 0) { 
                    // 1. 필요한 초기 선속도(v0) 계산 (Physics)
                    double v0 = Math.sqrt((g * x * x) / (2 * Math.pow(Math.cos(theta), 2) * discriminant));
                    
                    // 2. 선속도(v0) -> 모터 TPS 변환 (Engineering Correction)
                    double requiredTPS = calculateTPSFromVelocity(v0);
                    
                    setVelocity(requiredTPS);
                } else {
                    // 물리적으로 도달 불가능한 위치일 경우 기본값 혹은 정지
                    gamepad.rumble(500); // 경고 진동
                    setVelocity(DEFAULT_TPS); 
                }

            } else { 
                setVelocity(DEFAULT_TPS); 
            }
        } else { 
            setPower(0); 
        }

        // 수동 미세 조정 (Feedforward Bias 조정 느낌으로 활용 가능)
        if      (gamepad.dpadUpWasPressed())   { addTPS(10);}
        else if (gamepad.dpadDownWasPressed()) { addTPS(-10);}
        else if (gamepad.dpadLeftWasPressed()) { addTPS(1);}
        else if (gamepad.dpadRightWasPressed()){ addTPS(-1);}

        // 서보 회전
        if (gamepad.yWasPressed()) {
            isServoSpin = !isServoSpin;
            if (isServoSpin) {
                servoSpin(SERVO_MAX);
            } else {
                servoSpin(SERVO_MIN);
            }
        }
    }

    /**
     * 이론적 선속도(v0)를 모터의 목표 TPS로 변환합니다.
     * 공식: TPS = (v0 / (r * efficiency)) * (ticks_per_rev / (2 * pi)) * gear_ratio
     * @param v0Required 필요한 초기 발사 속도 (in/s)
     * @return Motor Ticks Per Second
     */
    private double calculateTPSFromVelocity(double v0Required) {
        // 실제 휠이 내야 하는 접선 속도 (미끄러짐 등 비효율 고려)
        double wheelTangentialVelocity = v0Required / SHOOTER_EFFICIENCY; 

        // 필요한 각속도 (rad/s) = v / r
        double angularVelocity = wheelTangentialVelocity / FLYWHEEL_RADIUS;

        // Rad/s -> TPS 변환
        // 1회전 = 2pi rad = TICKS_PER_REV
        double tps = angularVelocity * (TICKS_PER_REV / (2 * Math.PI)) * GEAR_RATIO;

        return tps;
    }

    public void setVelocity(double TPS) {
        setTPS(TPS);
        shooterLeft.setVelocity(this.currentTargetTPS);
        shooterRight.setVelocity(this.currentTargetTPS);
    }
    
    public void addVelocity(double amount) {
        setVelocity(this.currentTargetTPS + amount);
    }

    public void setPower(double power) {
        shooterLeft.setPower(power);
        shooterRight.setPower(power);
    }

    private void setTPS(double TPS) {
        // 모터 한계치 보호 (예: 3000 TPS)
        if (TPS > 3000)     { this.currentTargetTPS = 3000; }
        else if (TPS < 0)   { this.currentTargetTPS = 0; }
        else                { this.currentTargetTPS = TPS; }
    }
    
    public void addTPS(double amount) {
        setTPS(currentTargetTPS + amount);
    }

    // servo
    public void servoSpin(double power) {
        servo.setPower(power);
    }
}