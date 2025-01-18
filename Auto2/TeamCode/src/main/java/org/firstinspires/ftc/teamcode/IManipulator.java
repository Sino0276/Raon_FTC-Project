package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public interface IManipulator {

    void initialize(LinearOpMode linearOpMode);

//    void postStartInitialize();

//    void setSpeeds(double forward, double sideways, double rotate);

//    void move

    void angle(double degrees, double max_speed);

    /**
     *
     * @param len 0.0 ~ 1.0
     * @param max_speed 0.0 ~ 1.0
     */
    void length(double len, double max_speed);
}
