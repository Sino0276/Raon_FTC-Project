package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretSubsystem extends SubsystemBase {

    private MotorEx turretMotor;

    // PIDF
    public static double
            kP = 0,
            kI = 0,
            kD = 0,
            kS = 0,
            kV = 0;


    public TurretSubsystem(HardwareMap hardwareMap, String motorName) {
        // 모터 초기화
        turretMotor = new MotorEx(hardwareMap, motorName, Motor.GoBILDA.RPM_312);

        // 모터 반전 (모터의 회전 방향이 반대라면 수정)
        turretMotor.setInverted(false);

        // 모터 모드
        turretMotor.setRunMode(Motor.RunMode.VelocityControl);

        // PIDF 계수 설정
        updateCoefficients();

        // ZeroPowerBehavior
        turretMotor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }

    public void updateCoefficients() {
        turretMotor.setVeloCoefficients(kP, kI, kD);
        turretMotor.setFeedforwardCoefficients(kS, kV);
    }

    public void turn(double angle) {

    }

    public void stop() {
        turretMotor.stopMotor();
    }
}
