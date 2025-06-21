package org.firstinspires.ftc.teamcode.mecanum.manipulator;

import org.firstinspires.ftc.teamcode.mecanum.IMecanum;

public interface IManipulation extends IMecanum {

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
