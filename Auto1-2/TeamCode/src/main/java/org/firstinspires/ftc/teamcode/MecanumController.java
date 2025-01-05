package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class MecanumController extends Utilities {
	private autoraonJava2 main;
	private Telemetry telemetry;
	private double power = 0.6;
	private double turnSpeed = 0.4;
	private double denominator;
	private DcMotor left1;
	private DcMotor left2;
	private DcMotor right1;
	private DcMotor right2;

	public MecanumController
			(autoraonJava2 main, DcMotor left1, DcMotor left2, DcMotor right1, DcMotor right2) {
		this.main = main;
		this.left1 = left1;
		this.left2 = left2;
		this.right1 = right1;
		this.right2 = right2;

		telemetry = main.telemetry;

		_initMotor();
	}
	private void _initMotor() {
		// 모터 방향 초기화
		left1.setDirection(DcMotor.Direction.FORWARD);
		left2.setDirection(DcMotor.Direction.FORWARD);
		right1.setDirection(DcMotor.Direction.REVERSE);
		right2.setDirection(DcMotor.Direction.REVERSE);
		// TargetPosition 오차 설정
		((DcMotorEx) left1).setTargetPositionTolerance(10);
		((DcMotorEx) left2).setTargetPositionTolerance(10);
		((DcMotorEx) right1).setTargetPositionTolerance(10);
		((DcMotorEx) right2).setTargetPositionTolerance(10);
		// 모터 고정
		left1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		left2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		right1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		right2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		// 모드 초기화(바퀴)
		left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		// 모드 초기화(팔)
//		multiTelemetry.addData("모터 초기화 됨", "대기중...");
//		multiTelemetry.update();
	}

	private void turn(double output) {
		left1.setPower(-(turnSpeed * output));
		left2.setPower(-(turnSpeed * output));
		right1.setPower(turnSpeed * output);
		right2.setPower(turnSpeed * output);
	}

	public void mecanmTurn(double targetAngle) {
		main.imu.resetYaw();
		main.PIDCtrl.resetPID();
		// rx = -(Math.abs(power) * (targetAngle / Math.abs(targetAngle)));
		while (Math.abs(angleWrap(targetAngle - main.IMUCtrl.getYaw())) > 1) {
			main.IMUCtrl.callIMU();
			turn(main.PIDCtrl.PIDControl(targetAngle, main.IMUCtrl.getYaw()));
		}
		left1.setPower(0);
		left2.setPower(0);
		right1.setPower(0);
		right2.setPower(0);
	}

	/**
	 * double x = x_targetDist
	 * double y = y_targetDist
	 * targetAngle
	 * targetAngle값을 잘 지정하면 포물선으로 갈지도?
	 */
	public void mecanumMove(double x, double y, double targetAngle) {
		double dist = Math.sqrt(x * x + y * y);
		double error = dist - left1.getCurrentPosition();
		double rx = 0;

		denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));

		// 인코더 계산을 위한 용도로 초기화
		resetMotor();
		main.PIDCtrl.resetPID(); /////////// 확인
		// 바퀴 4개의 인코더 값의 합 / 4 == 바퀴 4개 인코더 값의 평균
		// 평균 - 목표거리 == 오차
		// 오차의 절대값 > 허용오차(1인치)
		while (/*opModeIsActive() && */Math.abs(error) >= 10) {
			main.IMUCtrl.callIMU();
			error = dist - left1.getCurrentPosition();

			rx = main.PIDCtrl.PIDControl(targetAngle, main.IMUCtrl.getYaw());
			denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));

			left1.setPower(((((y + x) - left1.getCurrentPosition()) + rx) / denominator) + 0.1);
			left2.setPower(((((y - x) - left2.getCurrentPosition()) - rx) / denominator) + 0.1);
			right1.setPower(((((y - x) - right1.getCurrentPosition()) + rx) / denominator) + 0.1);
			right2.setPower(((((y + x) - right2.getCurrentPosition()) - rx) / denominator) + 0.1);

//			tmt.addData("left1", left1.getCurrentPosition());
//			tmt.update();
		}
		left1.setPower(0);
		left2.setPower(0);
		right1.setPower(0);
		right2.setPower(0);
	}

	public void alignWithAprilTag(AprilTagDetection tag) {
		if (main.aprilTag.id == tag.id) {
			mecanumMove(tag.ftcPose.x, 0, 0);
			mecanmTurn(tag.ftcPose.bearing);
		}
	}

	/**
	 * 모터 인코더 리셋
	 */
	private void resetMotor() {
		int EncoderAvg;

		// 모드 초기화(바퀴)
		left1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		left2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		right1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		right2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		left1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		left2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		right1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		right2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		// 평균값
		EncoderAvg = (left1.getCurrentPosition() + left2.getCurrentPosition() + right1.getCurrentPosition() + right2.getCurrentPosition()) / 4;
	}

}
