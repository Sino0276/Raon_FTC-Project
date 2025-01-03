package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class HandController {
	private autoraonJava2 main;
	private Telemetry telemetry;
	Servo finger;
	Servo hand;

	public HandController(autoraonJava2 main, Telemetry telemetry, Servo finger, Servo hand) {
		this.main = main;
		this.telemetry = telemetry;
		this.finger = finger;
		this.hand = hand;
	}

	public void movefinger(boolean value) {
		if (value) { finger.setPosition(1); }   // true     먹기
		else { finger.setPosition(0); }         // false    뱉기 (퉤!)
	}

	/**값 조절 할 것!**/
	public void moveHand(boolean value) {
		if (value) { hand.setPosition(1); }     // true     돌리기
		else { hand.setPosition(0); }           // false    원점
	}
}
