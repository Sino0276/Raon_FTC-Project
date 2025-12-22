package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;
// 지금당장떠나면아무도다치지않는다그러지않으면너희는모두죽어탐정놀이도이젠끝이다현실로돌아가면잊지말고전해라스텔라론헌터가너희의마지막을배웅했다는것을소탕시작액션원집행목표고정즉시처단프로토콜통과초토화작전집행깨어갔군한참이나기다렸다우린전에만난적이있지난스텔라론헌터샘이다일찍이네앞에나타나사실을알려주고싶었어하지만예상보다방해물이많군열한차례시도했지만모두실패로끝났지그러는사이에나도모르게이세계와긴밀이연결되어각본의구속에서벗어날수없게됐다엘리오말대로우리는이꿈의땅에서잊을수없는수확을얻게될테지나에겐그와카프카처럼사람의마음을꿰뚫어보는통찰력도은랑과블레이드처럼뛰어난특기도없다내가잘하는것들대부분은불쌍히여길필요없는악당에게만적용되지그러니내가사용할수있는수단도단하나뿐이다네게보여주기위한거야내전부를반딧불이처럼죽음을각오하고불속에뛰어들며살거야깨어난현실에서다시만나길

@Config
public class FlywheelSubsystem extends SubsystemBase {

    private MotorEx flywheelMotor;

    // PIDF
    public static double kP = 0,
                         kI = 0,
                         kD = 0,
                         kS = 0,
                         kV = 0;

    // RPM 도달 허용 오차
    public static double VELOCITY_TOLERANCE = 50.0;

    private double targetRPM = 0.0;

    /**
     * 모터 초기화 및 설정
     * @param hardwareMap
     */
    public FlywheelSubsystem(HardwareMap hardwareMap, String motorName, Motor.GoBILDA motorType) {
        // 플라이휠 초기화
        flywheelMotor = new MotorEx(hardwareMap, motorName, motorType);

        // 모터 반전 (모터의 회전 방향이 반대라면 수정)
        flywheelMotor.setInverted(false);

        // 모터 모드
        flywheelMotor.setRunMode(Motor.RunMode.VelocityControl);

        // PIDF 계수 설정
        updateCoefficients();

        // ZeroPowerBehavior
        flywheelMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);  // 굳이 플라이휠에 무리를 줄 필요는 없음
    }

    /**
     * PIDF 계수 업데이트
     */
    public void updateCoefficients() {
        flywheelMotor.setVeloCoefficients(kP, kI, kD);
        flywheelMotor.setFeedforwardCoefficients(kS, kV);
    }

    /**
     * 플라이휠 속도 제어
     * @param rpm
     */
    public void shoot(double rpm) {
        // PIDF계수 재설정
        updateCoefficients();

        rpm = Range.clip(rpm, -flywheelMotor.getMaxRPM(), flywheelMotor.getMaxRPM());
        this.targetRPM = rpm;

        // RPM -> TPS (Ticks Per Second) 변환 공식
        // TPS = (RPM * CPR) / 60
        double targetTPS = (rpm * flywheelMotor.getCPR()) / 60.0;

        flywheelMotor.setVelocity(targetTPS);
    }

    /**
     * 플라이휠 정지
     */
    public void stop() {
        shoot(0);
        flywheelMotor.stopMotor();
    }

    /**
     * 플라이휠이 목표 RPM에 도달했는지 확인
     * 발사 가능 유무
     * @return
     */
    public boolean isReady() {
        // [변경됨] CPR 값을 직접 호출하여 계산
        double cpr = flywheelMotor.getCPR();
        double currentLeftRPM = (flywheelMotor.getCorrectedVelocity() * 60) / cpr;
        double currentRightRPM = (flywheelMotor.getCorrectedVelocity() * 60) / cpr;

        double avgRPM = (Math.abs(currentLeftRPM) + Math.abs(currentRightRPM)) / 2.0;

        return targetRPM > 0 && Math.abs(targetRPM - avgRPM) <= VELOCITY_TOLERANCE;
    }
}
