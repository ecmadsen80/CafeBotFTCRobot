package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.ArrayList;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="MotorTestProgram OpMode", group="Linear OpMode")

public class MotorTestProgram extends LinearOpMode {

    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftDrive = null;
    private DcMotorEx leftTurn = null;
    private DcMotorEx rightDrive = null;
    private DcMotorEx rightTurn = null;
    private Servo light;

    private Gamepad gp1;
    ArrayList<Float> myList = new ArrayList<>();
    static final double TURN_TICKS_PER_REV = 751.8; //gobuilda 5204-8002-0027
    static final double TURN_POWER = 1;

    // Max Velocity (in TPS) = (Max RPM * PPR for that gearing)/60
    // for GoBuilda 5204 223 RPM motor with 751.8 PPR = 2794.19
    // AI suggested that max is typically around 2500 (90% of max) for "headroom" for
    // the PID controller
    static final double TURN_VELOCITY = 1000; //i set to 1000 to move slower
    static final double DEADZONE = 0.1;
    private boolean goingUp = false;
    private double pos = 0.284;

    @Override
    public void runOpMode() {

        leftDrive  = hardwareMap.get(DcMotorEx.class, "left_drive");
        leftTurn   = hardwareMap.get(DcMotorEx.class, "left_turn");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        rightTurn  = hardwareMap.get(DcMotorEx.class, "right_turn");
        light = hardwareMap.get(Servo.class, "light");

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightTurn.setDirection(DcMotor.Direction.REVERSE);


        leftTurn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightTurn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        leftTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftTurn.setTargetPosition(0);
        rightTurn.setTargetPosition(0);

        leftTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        gp1 = gamepad1;

        waitForStart();

        runtime.reset();
        leftTurn.setPower(1.0);
        rightTurn.setPower(1.0);
        int position= 0;
        double targetAngle = 0;

        while (opModeIsActive()) {
            /*
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y; // FTC Y inverted

            // Raw magnitude
            double rawMag = Math.hypot(x, y);

            // Deadzone
            if (rawMag < DEADZONE) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                leftTurn.setVelocity(0); //setVelocity actively holds the motor in it's current position
                rightTurn.setVelocity(0);
                continue;
            }

            // Normalize stick vector
            double nx = x / rawMag;
            double ny = y / rawMag;

            // Scaled magnitude (optional throttle curve)
            double mag = Range.clip(rawMag, 0, 1);

            // Drive power where "mag" is a clipped version of "rawMag"
            leftDrive.setPower(mag);
            rightDrive.setPower(mag);

            // Angle in degrees [0,360)
            double angleDeg = Math.toDegrees(Math.atan2(ny, nx));
            if (angleDeg < 0) angleDeg += 360;
            //0 is directly right (1, 0), 90 is directly up (0, 1), 180 is directly left (-1, 0),
            // and 270 is directly down (-1,0)
            angleDeg += 0;

             */

            if (gamepad1.yWasPressed()) targetAngle = 0;
            else if (gamepad1.bWasPressed()) targetAngle = 90;
            else if (gamepad1.aWasPressed()) targetAngle = 180;
            else if (gamepad1.xWasPressed()) targetAngle = 270;

            double angleDeg = targetAngle; //left this in so when I revert to original code it'll
            //still be there to use below

            // Correct degrees → encoder ticks
            int targetTicks = (int)((angleDeg / 360.0) * TURN_TICKS_PER_REV);

            // Get current encoder positions
            int leftCurrentTicks = leftTurn.getCurrentPosition();
            int rightCurrentTicks = rightTurn.getCurrentPosition();
            // Convert to degrees
            int leftCurrentDegrees = (int)((leftCurrentTicks / (double)TURN_TICKS_PER_REV) * 360);
            int rightCurrentDegrees = (int)((rightCurrentTicks / (double)TURN_TICKS_PER_REV) * 360);



            // Turn motors
            leftTurn.setTargetPosition(targetTicks);
            rightTurn.setTargetPosition(targetTicks);

            //using setVelocity should use the internal PID controller
            leftTurn.setVelocity(TURN_VELOCITY);
            rightTurn.setVelocity(TURN_VELOCITY);

            // Telemetry
            telemetry.addData("Angle deg", angleDeg);
            telemetry.addData("Target ticks", targetTicks);
            telemetry.addData("Left Current Position", leftCurrentDegrees);
            telemetry.addData("Right Current Position", rightCurrentDegrees);
            //telemetry.addData("Raw mag", rawMag);
            //telemetry.addData("Mag", mag);
            //telemetry.addData("XY", "%.2f %.2f", x, y);
            telemetry.update();

        }

    }

}
