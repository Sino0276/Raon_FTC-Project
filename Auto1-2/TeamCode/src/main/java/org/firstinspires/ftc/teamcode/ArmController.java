package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class ArmController {
	private autoraonJava1 main;
	int armAngleRev;    // 팔이 한바퀴 돌기 위해 필요한 인코더 값
	double armLengthSpeed = 1.0;
	double armAngleSpeed = 1.0;
	DcMotor armLength;
	DcMotor armAngle;

	public ArmController
			(autoraonJava1 main, int armLengthRev, double lengthSpeed, double angleSpeed, DcMotor armLength, DcMotor armAngle) {
		this.main = main;
		this.armAngleRev = armLengthRev;
		this.armLengthSpeed = lengthSpeed;
		this.armAngleSpeed = angleSpeed;
		this.armLength = armLength;
		this.armAngle = armAngle;

		_initArm();
	}

	private void _initArm() {
		// 모터 방향 초기화
		armAngle.setDirection(DcMotor.Direction.REVERSE);
		armLength.setDirection(DcMotor.Direction.REVERSE);
		// 모터 고정
		armAngle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		armLength.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		// 모드 초기화(팔)
		armAngle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		armLength.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		armAngle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		armLength.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		telemetry.addData("팔 초기화 됨", "대기중...");
		telemetry.update();
	}

	public void setArmLength(int encoder) {
		armLength.setTargetPosition(encoder);
	}

	/**0 ~ 13425 (360도)
	 * 6712.5 (180도)
	 * 3356.25 (90도)
	 * . . .**/
	public void setArmAngle(int angle) {
		int encoder = armAngleRev * angle / 360;
		armAngle.setTargetPosition(encoder);
	}


}
