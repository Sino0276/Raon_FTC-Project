package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.teamcode.PID.PIDController;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class MecanumController {
	// 인치당 인코더
	double COUNTS_PER_MOTOR_REV = 537;
	double DRIVE_GEAR_REDUCTION = 1;
	double WHEEL_DIAMETER_INCHES = 3.7;
	public double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
	double power = 0.6;
	double turnSpeed = 0.4;
	double denominator;
	DcMotor left1;
	DcMotor left2;
	DcMotor right1;
	DcMotor right2;

	autoraonJava1 main = new autoraonJava1();

	public MecanumController
			(DcMotor left1, DcMotor left2, DcMotor right1, DcMotor right2, double encoder_Per_MotorRev, double driveGear_Reduction, double wheel_Inch) {
		this.left1 = left1;
		this.left2 = left2;
		this.right1 = right1;
		this.right2 = right2;
		this.COUNTS_PER_MOTOR_REV = encoder_Per_MotorRev;
		this.DRIVE_GEAR_REDUCTION = driveGear_Reduction;
		this.WHEEL_DIAMETER_INCHES = wheel_Inch;
	}
	public void _initMotor() {
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
		telemetry.addData("모터 초기화 됨", "대기중...");
		telemetry.update();
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
		while (Math.abs(angleWrap(targetAngle - main.IMUCtrl.yaw)) > 1) {
			main.IMUCtrl.callIMU();
			turn(main.PIDCtrl.PIDControl(targetAngle, main.IMUCtrl.yaw));
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

			rx = main.PIDCtrl.PIDControl(targetAngle, main.IMUCtrl.yaw);
			denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));

			left1.setPower(((((y + x) - left1.getCurrentPosition()) + rx) / denominator) + 0.2);
			left2.setPower(((((y - x) - left2.getCurrentPosition()) - rx) / denominator) + 0.2);
			right1.setPower(((((y - x) - right1.getCurrentPosition()) + rx) / denominator) + 0.2);
			right2.setPower(((((y + x) - right2.getCurrentPosition()) - rx) / denominator) + 0.2);

			telemetry.addData("left1", left1.getCurrentPosition());
			telemetry.update();
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

	private double angleWrap(double wrappingAngle) {
		while (wrappingAngle > 180) {
			wrappingAngle = wrappingAngle - 360;
		}
		while (wrappingAngle < -180) {
			wrappingAngle = wrappingAngle + 360;
		}
		return wrappingAngle;
	}

	public int inchToEncoder(double inch) { return (int)(inch * COUNTS_PER_INCH); }     // 강제 형 변환
}
