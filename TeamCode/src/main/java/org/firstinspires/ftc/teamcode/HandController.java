package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

public class HandController {
	Servo finger;
	Servo hand;

	public HandController(Servo finger, Servo hand) {
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
