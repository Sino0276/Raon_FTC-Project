package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import static org.firstinspires.ftc.teamcode.Utilities.*;

@Autonomous(name = "autoraonJava1")
public class autoraonJava2 extends LinearOpMode{
	MultipleTelemetry tmt = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

	private DcMotor left1;
	private DcMotor left2;
	private DcMotor right1;
	private DcMotor right2;
	private DcMotor arm_angle;
	private DcMotor arm_length;
	private Servo finger;
	private Servo hand;

	public IMU imu;
	List<AprilTagDetection> myAprilTagDetections;
	boolean isParking;
	boolean isBasket;
	AprilTagDetection aprilTag;
	AprilTagProcessor myAprilTagProcessor;

	boolean USE_WEBCAM;

	int sleepTime;

	private void _init() {
		_initHardWareMap();
		_initAprilTag();
		telemetry.update();

		mecanumCtrl = new MecanumController(this, left1, left2, right1, right2, 537, 1, 3.7);
		armCtrl = new ArmController(this, 13425, 1.0, 0.8, arm_length, arm_angle);
		handCtrl = new HandController(this, hand, finger);
		PIDCtrl = new PIDController(this);
		IMUCtrl = new IMUController(this, imu);
	}

	private void _initHardWareMap() {
		left1 = hardwareMap.get(DcMotor.class, "left1");
		left2 = hardwareMap.get(DcMotor.class, "left2");
		right1 = hardwareMap.get(DcMotor.class, "right1");
		right2 = hardwareMap.get(DcMotor.class, "right2");
		imu = hardwareMap.get(IMU.class, "imu");
		arm_angle = hardwareMap.get(DcMotor.class, "arm_angle");
		arm_length = hardwareMap.get(DcMotor.class, "arm_length");
		hand = hardwareMap.get(Servo.class, "hand");
		finger = hardwareMap.get(Servo.class, "finger");
	}

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

	private void aprilEx() {
		// Get a list of AprilTag detections.
		myAprilTagDetections = myAprilTagProcessor.getDetections();
		// Iterate through list and call a function to display info for each recognized AprilTag.
		for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
			aprilTag = myAprilTagDetection_item;
			if (aprilTag.metadata != null) {
				telemetry.addLine("" + JavaUtil.formatNumber(aprilTag.ftcPose.yaw, 6, 1));
			}
		}
	}

	MecanumController mecanumCtrl;
	ArmController armCtrl;
	HandController handCtrl;
	PIDController PIDCtrl;
	IMUController IMUCtrl;

	@Override
	public void runOpMode() {
		isParking = false;
		isBasket = false;
		// 카메라 사용여부
		USE_WEBCAM = true;
		// 초기화
		_init();
		telemetry.update();

		waitForStart();
		if (opModeIsActive()) {
			while (opModeIsActive()) { // 반복
				IMUCtrl.callIMU();

				autonomus();

				telemetry.update();
			}
		}
	}

	private void autonomus() {
		if (isBasket == false) {
			mecanumCtrl.mecanumMove(0, mecanumCtrl.inchToEncoder(24), 0);
			sleep(sleepTime);
			mecanumCtrl.mecanmTurn(90); ///////////////////// 확인
			sleep(sleepTime);
		}

		myAprilTagDetections = myAprilTagProcessor.getDetections();
		for (AprilTagDetection item : myAprilTagDetections) {
			aprilTag = item;
			if (aprilTag.metadata != null) {

				if (isParking == false) {
					if (isBasket == false) {
						isBasket = true;

						scoring_basket(aprilTag);
						continue;
					}
					else {
						isParking = true;
						parking(aprilTag);
					}
				}
			}
		}
	}

	/**이 함수는 끝나면 11 or 14 태그를 바라본다**/
	private void scoring_basket(AprilTagDetection tag) {
		if (tag.id == 13 || tag.id == 16) {
			IMUCtrl.callIMU();


			mecanumCtrl.mecanumMove(0, mecanumCtrl.inchToEncoder(tag.ftcPose.range - 24), 0);
			sleep(sleepTime);
			mecanumCtrl.alignWithAprilTag(tag);
			mecanumCtrl.mecanmTurn(45);
			// 팔 움직이는 코드
			sleep(sleepTime);
			//
			armCtrl.setArmAngle(180);
			sleep(1000);
			handCtrl.moveHand(false);		// 수정하기
			sleep(500);
			arm_angle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
			sleep(100);
			arm_angle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
			handCtrl.movefinger(true);
			sleep(500);
			armCtrl.setArmAngle(10);
			sleep(2000);

			// ↖ : ←  ↓
			mecanumCtrl.mecanmTurn(5 + (45+90));
		}
	}

	/**이 함수는 끝나면 11 or 14 태그를 바라본다**/
	private void parking(AprilTagDetection tag) {
		if (tag.id == 11 || tag.id == 14) {
			IMUCtrl.callIMU();
			// 테스트 해보고 거리조절
			mecanumCtrl.mecanumMove(0, mecanumCtrl.inchToEncoder(tag.ftcPose.range - 10), tag.ftcPose.bearing);
			sleep(sleepTime);
			mecanumCtrl.alignWithAprilTag(tag);
			sleep(sleepTime);
			// X값이 -인지 확인하기
			mecanumCtrl.mecanumMove(mecanumCtrl.inchToEncoder(20), 0, 0);
		}
	}
}
