package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "autoraonJava1")
public class autoraonJava1 extends LinearOpMode {
	MultipleTelemetry tmt = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

	private DcMotor left1;
	private DcMotor left2;
	private DcMotor right1;
	private DcMotor right2;
	private DcMotor arm_angle;
	private DcMotor arm_length;

	private IMU imu;
	double yaw;
	List<AprilTagDetection> myAprilTagDetections;
	boolean isParking;
	AprilTagDetection myAprilTagDetection;
	AprilTagProcessor myAprilTagProcessor;

	double COUNTS_PER_INCH;

	double rx;

	boolean USE_WEBCAM;

	double power;
	double turnSpeed;
	double denominator;
	double Kp;
	double Ki;
	double Kd;
	double integralSum;
	double lastError;
	ElapsedTime timer;
	double weight;

	private void _initAprilTag() {
		AprilTagProcessor.Builder myAprilTagProcessorBuilder;
		VisionPortal.Builder myVisionPortalBuilder;
		VisionPortal myVisionPortal;

		myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
		myAprilTagProcessor = myAprilTagProcessorBuilder.build();
		myVisionPortalBuilder = new VisionPortal.Builder();
		if (USE_WEBCAM) {
			// Use a webcam.
			myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "camera"));
		} else {
			// Use the device's back camera.
			myVisionPortalBuilder.setCamera(BuiltinCameraDirection.FRONT);
		}
		myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
		myVisionPortal = myVisionPortalBuilder.build();
		telemetry.addData("AprilTag 초기화 됨", "대기중...");
		telemetry.update();
	}

	/**
	 * 모터 관련 여러기능 초기화
	 */
	private void _initMotor() {
		// 모터 방향 초기화
		left1.setDirection(DcMotor.Direction.FORWARD);
		left2.setDirection(DcMotor.Direction.FORWARD);
		right1.setDirection(DcMotor.Direction.REVERSE);
		right2.setDirection(DcMotor.Direction.REVERSE);
		arm_angle.setDirection(DcMotor.Direction.REVERSE);
		arm_length.setDirection(DcMotor.Direction.REVERSE);
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
		arm_angle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		arm_length.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
		arm_angle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		arm_length.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		arm_angle.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		arm_length.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		telemetry.addData("모터 초기화 됨", "대기중...");
		telemetry.update();
	}

	private void _initIMU() { //                                                                     로고 방향                                                 USB포트 방향
		imu.initialize(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.LEFT)));
		imu.resetYaw();
		telemetry.addData("IMU 초기화 됨", "시작 대기중");
	}

	private void _initPID() {
		// PID변수
		Kp = PIDConstants.Kp;
		Ki = PIDConstants.Ki;
		Kd = PIDConstants.Kd;
		integralSum = 0;
		lastError = 0;
		timer = new ElapsedTime();
	}

	private void resetPID() {
		// PID변수
		integralSum = 0;
		lastError = 0;
		timer.reset();
	}

	/**
	 * 모터 관련 여러기능 초기화
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

	private void callIMU() {
		YawPitchRollAngles orientation;
		AngularVelocity angularVelocity;

		orientation = imu.getRobotYawPitchRollAngles();
		angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
		yaw = orientation.getYaw(AngleUnit.DEGREES);
	}

	private void turnBot(double output) {
		left1.setPower(-(turnSpeed * output));
		left2.setPower(-(turnSpeed * output));
		right1.setPower(turnSpeed * output);
		right2.setPower(turnSpeed * output);
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

	/**
	 * https://www.ctrlaltftc.com/the-pid-controller
	 */
	private double PIDControl(double reference, double now) {
		double error;
		double derivative;
		double out;

		tmt.addData("목표 IMU Angle", reference);
		tmt.addData("현재 IMU Angle", now);

		error = angleWrap(reference - now);
		derivative = (error - lastError) / timer.milliseconds();
		integralSum += error * timer.milliseconds();
		out = -(Kp * error + Ki * integralSum + Kd * derivative);

		lastError = error;
		timer.reset();

		tmt.update();

		return out;
	}

	private void mecanmTurnCCW(double targetAngle) {
		imu.resetYaw();
		resetPID();
		// rx = -(Math.abs(power) * (targetAngle / Math.abs(targetAngle)));
		while (Math.abs(angleWrap(targetAngle - yaw)) > 2) {
			callIMU();
			turnBot(PIDControl(targetAngle, yaw));
		}
		left1.setPower(0);
		left2.setPower(0);
		right1.setPower(0);
		right2.setPower(0);
	}

	private void telemetryAprilTag() {
		// Get a list of AprilTag detections.
		myAprilTagDetections = myAprilTagProcessor.getDetections();
		// Iterate through list and call a function to display info for each recognized AprilTag.
		for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
			myAprilTagDetection = myAprilTagDetection_item;
			if (myAprilTagDetection.metadata != null) {
				telemetry.addLine("" + JavaUtil.formatNumber(myAprilTagDetection.ftcPose.yaw, 6, 1));
			}
		}
	}

	/**
	 * dist == 인코더 값
	 * CONUTS_PER_INCH * n = n인치 이동 가량의 인코더 값
	 */
	private void Move2(double x, double y, double dist, double targetAngle) {
		x = Math.abs(power) * x;
		y = Math.abs(power) * y;
		rx = 0;
		denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
		// 인코더 계산을 위한 용도로 초기화
		resetMotor();
		resetPID();
		// 바퀴 4개의 인코더 값의 합 / 4 == 바퀴 4개 인코더 값의 평균
		// 평균 - 목표거리 == 오차
		// 오차의 절대값 > 허용오차(1인치)
		while (opModeIsActive() && Math.abs(left1.getCurrentPosition() - dist) > COUNTS_PER_INCH) {
			callIMU();
			rx = PIDControl(targetAngle, yaw);
			denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));
			left1.setPower((Range.clip((y + x) * -Kp * (left1.getCurrentPosition() - dist), -power, power) + rx) / denominator);
			left2.setPower((Range.clip((y - x) * -Kp * (left1.getCurrentPosition() - dist), -power, power) - rx) / denominator);
			right1.setPower((Range.clip((y - x) * -Kp * (left1.getCurrentPosition() - dist), -power, power) + rx) / denominator);
			right2.setPower((Range.clip((y + x) * -Kp * (left1.getCurrentPosition() - dist), -power, power) - rx) / denominator);
			telemetry.addData("l1", left1.getCurrentPosition());
			telemetry.update();
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
	private void Move3(double x, double y, double targetAngle) {
		double dist = Math.sqrt(x * x + y * y);
		double error = left1.getCurrentPosition() - dist;
		x = Math.abs(power) * x;
		y = Math.abs(power) * y;
		rx = 0;

		// 인코더 계산을 위한 용도로 초기화
		resetMotor();
		resetPID(); /////////// 확인
		// 바퀴 4개의 인코더 값의 합 / 4 == 바퀴 4개 인코더 값의 평균
		// 평균 - 목표거리 == 오차
		// 오차의 절대값 > 허용오차(1인치)
		while (opModeIsActive() && Math.abs(error) > COUNTS_PER_INCH) {
			callIMU();
			error = left1.getCurrentPosition() - dist;

			rx = PIDControl(targetAngle, yaw);
			denominator = JavaUtil.maxOfList(JavaUtil.createListWith(JavaUtil.sumOfList(JavaUtil.createListWith(Math.abs(y), Math.abs(x), Math.abs(rx))), 1));  // (x + y + rx) 값과 1중에서 더 높은 값을 반환

			left1.setPower((((y + x) * -Kp * error) + rx) / denominator);
			left2.setPower((((y - x) * -Kp * error) - rx) / denominator);
			right1.setPower((((y - x) * -Kp * error) + rx) / denominator);
			right2.setPower((((y + x) * -Kp * error) - rx) / denominator);

			telemetry.addData("left1", left1.getCurrentPosition());
			telemetry.update();
		}
		left1.setPower(0);
		left2.setPower(0);
		right1.setPower(0);
		right2.setPower(0);
	}

	private void alignWithAprilTag(AprilTagDetection aprilTag) {
		if (myAprilTagDetection.id == aprilTag.id) {
			Move3(aprilTag.ftcPose.x, 0, aprilTag.ftcPose.bearing);
		}
	}

	@Override
	public void runOpMode() {
		// 인치당 인코더
		double COUNTS_PER_MOTOR_REV = 537;
		double DRIVE_GEAR_REDUCTION = 1;
		double WHEEL_DIAMETER_INCHES = 3.7;
		COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);

		left1 = hardwareMap.get(DcMotor.class, "left1");
		left2 = hardwareMap.get(DcMotor.class, "left2");
		right1 = hardwareMap.get(DcMotor.class, "right1");
		right2 = hardwareMap.get(DcMotor.class, "right2");
		imu = hardwareMap.get(IMU.class, "imu");
		arm_angle = hardwareMap.get(DcMotor.class, "arm_angle");
		arm_length = hardwareMap.get(DcMotor.class, "arm_length");

		isParking = false;
		// 카메라 사용여부
		USE_WEBCAM = true;
		power = 0.4;
		turnSpeed = 0.4;
		weight = 0.055;

		// 초기화
		_initPID();
		_initAprilTag();
		_initMotor();
		_initIMU();
		telemetry.update();
		waitForStart();
		if (opModeIsActive()) {
			// Put run blocks here.
			while (opModeIsActive()) { // 반복
				callIMU();
				myAprilTagDetections = myAprilTagProcessor.getDetections();
				for (AprilTagDetection myAprilTagDetection_item2 : myAprilTagDetections) {
					myAprilTagDetection = myAprilTagDetection_item2;
					if (myAprilTagDetection.metadata != null) {
						if (isParking == false) {
							scoring_basket(myAprilTagDetection);
							parking(myAprilTagDetection);
						}
					}
				}
				telemetry.update();
			}
		}
	}

	private void scoring_basket(AprilTagDetection aprilTag) {
		if (aprilTag.id == 13 || aprilTag.id == 16) {
			Move3(0, COUNTS_PER_INCH * 24, 0);
			sleep(1000);
			mecanmTurnCCW(90); ///////////////////// 확인
			sleep(1000);
			Move3(0, aprilTag.ftcPose.range - (24 * COUNTS_PER_INCH), 0);
			sleep(1000);
			alignWithAprilTag(aprilTag);
			mecanmTurnCCW(45);
			// 팔 움직이는 코드
		}
	}

	private void parking(AprilTagDetection aprilTag) {
		if (myAprilTagDetection.id == 11 || myAprilTagDetection.id == 14) {
			isParking = true;
			resetMotor();
			resetPID();
			callIMU();

//      Move2(0, 1, COUNTS_PER_INCH * (myAprilTagDetection.ftcPose.range - 20), myAprilTagDetection.ftcPose.bearing);
//      sleep(1000);
//      // Y값이 -인지 확인하기
//      Move2(1, 0, COUNTS_PER_INCH * 25, 0);
			// 테스트 해보고 거리조절
			Move3(0, COUNTS_PER_INCH * (myAprilTagDetection.ftcPose.range - 20), myAprilTagDetection.ftcPose.bearing);
			sleep(1000);
			alignWithAprilTag(aprilTag);
			sleep(1000);
			// X값이 -인지 확인하기
			Move3(COUNTS_PER_INCH * 25, 0, 0);
		}
	}
}
