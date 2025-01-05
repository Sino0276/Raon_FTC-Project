package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Utilities.armAngleRev;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmController {
	private autoraonJava2 main;
	private Telemetry telemetry;
	private double armLengthSpeed = 1.0;
	private double armAngleSpeed = 0.8;
	private DcMotor armLength;
	private DcMotor armAngle;

	public ArmController
			(autoraonJava2 main, DcMotor armLength, DcMotor armAngle) {
		this.main = main;
		this.armLength = armLength;
		this.armAngle = armAngle;

		telemetry = main.telemetry;

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
//		telemetry.addData("팔 초기화 됨", "대기중...");
//		telemetry.update();
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
