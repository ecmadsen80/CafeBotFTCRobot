package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

//Take a page out of SLUH's book, by adding a Constants.java file.
//This will hold all of our important values and will be imported with every OpMode.
//This way, when we change these values it will change for every OpMode.
public class Constants {
    public static final int AS5600_ADDR = 0x36;
    public static final int ANGLE_REGISTER = 0x0E;

    public static final double LEFT_ZERO_POSITION = 3995 * 360 / 4096.0;
    public static final double RIGHT_ZERO_POSITION = 3874 * 360 / 4096.0;
    public static final double TICKS_PER_REV = 28;
    public static final double GOBILDA_ENCODER_TICKS = 751.8;
    public static final double TURN_VELOCITY = 2500;

    public static final double FLYWHEEL_DEFAULT = 2575;
    private static final PIDFCoefficients FLYWHEEL_PIDF = new PIDFCoefficients(100, 0.0, 0.0, 10);

}
