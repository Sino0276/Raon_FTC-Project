package org.firstinspires.ftc.teamcode.imReady;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

public class PusherHardware {
    public Servo servo;
    public List<Servo> servos;



    public PusherHardware(HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "servo");
//        servoSetup();
        servo.setDirection(Servo.Direction.FORWARD);

        servos = Arrays.asList(servo);
    }


//    private void servoSetup() {
//    }


}
