package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import java.lang.annotation.Annotation;

@Disabled
@Config
@TeleOp(name = "ServoTest", group = "Test")
public class ServoTest extends OpMode {

    public static double SERVO_MAX = 0.37;
    public static double SERVO_MIN = 0.085;

    private Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, "servo");
        servo.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void loop() {
        if (gamepad1.a) { servo.setPosition(SERVO_MAX); }
        else if (gamepad1.b) { servo.setPosition(SERVO_MIN); }
    }
}
