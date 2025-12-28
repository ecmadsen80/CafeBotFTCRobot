package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.ArrayList;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="MotorTestProgram OpMode", group="Linear OpMode")

public class MotorTestProgram extends LinearOpMode {

    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftDrive = null;
    private DcMotorEx leftTurn = null;
    private DcMotorEx rightDrive = null;
    private DcMotorEx rightTurn = null;
    private DcMotor leftPusher;
    private DcMotor rightPusher;
    private DcMotor intake;
    private Servo light;

    ArrayList<Float> myList = new ArrayList<>();
    static final double TURN_TICKS_PER_REV = 751.8; //gobuilda 5204-8002-0027
    static final double TURN_POWER = 1;

    // Max Velocity (in TPS) = (Max RPM * PPR for that gearing)/60
    // for GoBuilda 5204 223 RPM motor with 751.8 PPR = 2794.19
    // AI suggested that max is typically around 2500 (90% of max) for "headroom" for
    // the PID controller
    static final double TURN_VELOCITY = 2500; //i set to 1000 to move slower
    static final double DEADZONE = 0.1;
    private boolean goingUp = false;
    private double pos = 0.284;

    @Override
    public void runOpMode() {

        leftDrive  = hardwareMap.get(DcMotorEx.class, "left_drive");
        leftTurn   = hardwareMap.get(DcMotorEx.class, "left_turn");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        rightTurn  = hardwareMap.get(DcMotorEx.class, "right_turn");
        intake     = hardwareMap.get(DcMotor.class, "intake");
        leftPusher = hardwareMap.get(DcMotor.class, "leftpusher");
        rightPusher= hardwareMap.get(DcMotor.class, "rightpusher");
        light = hardwareMap.get(Servo.class, "light");
        //hi
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


        waitForStart();



        runtime.reset();
        leftTurn.setPower(1.0);
        rightTurn.setPower(1.0);
        int position= 0;
        double targetAngle = 0;

        while (opModeIsActive()) {

            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y; // FTC Y inverted
            double turn = gamepad1.right_stick_x;

            // Raw magnitude
            double rawMag = Math.hypot(x, y);



            double intakePower = gamepad1.left_trigger;
            if (intakePower > 0.1) {
                intake.setPower(-1.0);
                leftPusher.setPower(-1.0);
                rightPusher.setPower(-1.0);
            }




            // Normalize stick vector
            double nx = x / rawMag;
            double ny = y / rawMag;

            // Scaled magnitude (optional throttle curve)
            double mag = Range.clip(rawMag, 0, 1);

            // Drive power calculation with Turn (Z-axis rotation)
            double leftPower = mag + turn;
            double rightPower = mag - turn;

            // Use Range.clip to ensure power stays between -1 and 1
            leftDrive.setPower(Range.clip(leftPower, -1, 1));
            rightDrive.setPower(Range.clip(rightPower, -1, 1));

            // Angle in degrees [0,360)
            double angleDeg = Math.toDegrees(Math.atan2(nx, ny));
            if (angleDeg < 0) angleDeg += 360;
            //0 is directly right (1, 0), 90 is directly up (0, 1), 180 is directly left (-1, 0),
            // and 270 is directly down (-1,0)
            angleDeg += 0;


            /*
            This was here to test the specific angles and the PIDF controller
            if (gamepad1.yWasPressed()) targetAngle = 0;
            else if (gamepad1.bWasPressed()) targetAngle = 90;
            else if (gamepad1.aWasPressed()) targetAngle = 180;
            else if (gamepad1.xWasPressed()) targetAngle = 270;
            double angleDeg = targetAngle; //used for 4-button debug
            */

            //get current encoder positions
            int leftCurrentTicks = leftTurn.getCurrentPosition();
            int rightCurrentTicks = rightTurn.getCurrentPosition();
            // Convert to degrees
            int leftCurrentDegrees = (int)((leftCurrentTicks / (double)TURN_TICKS_PER_REV) * 360);
            int rightCurrentDegrees = (int)((rightCurrentTicks / (double)TURN_TICKS_PER_REV) * 360);


            //calculates the closest angle to the target angle and the sets the move to angle by adding to current degrees
            double moveToAngleLeft = closestAngle(angleDeg, leftCurrentDegrees) + leftCurrentDegrees;
            double moveToAngleRight = closestAngle(angleDeg, rightCurrentDegrees) + rightCurrentDegrees;


            // Correct degrees → encoder ticks
            int targetTicksRight = (int)((moveToAngleRight / 360.0) * TURN_TICKS_PER_REV);
            int targetTicksLeft = (int)((moveToAngleLeft / 360.0) * TURN_TICKS_PER_REV);

            // Turn motors
            leftTurn.setTargetPosition(targetTicksLeft);
            rightTurn.setTargetPosition(targetTicksRight);

            //using setVelocity should use the internal PID controller
            leftTurn.setVelocity(TURN_VELOCITY);
            rightTurn.setVelocity(TURN_VELOCITY);

            //leftTurn.setPower(TURN_POWER);
            //rightTurn.setPower(TURN_POWER);

            intake.setPower(0.0);
            leftPusher.setPower(0);
            rightPusher.setPower(0);

            // Telemetry
            telemetry.addData("IntakePower", intakePower);
            telemetry.addData("Angle deg", angleDeg);
            telemetry.addData("Target ticks", targetTicksLeft);
            telemetry.addData("Left Current Position", leftCurrentDegrees);
            telemetry.addData("Right Current Position", rightCurrentDegrees);
            //telemetry.addData("Raw mag", rawMag);
            //telemetry.addData("Mag", mag);
            //telemetry.addData("XY", "%.2f %.2f", x, y);
            telemetry.update();

            // Deadzone
            if (rawMag < DEADZONE) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                leftTurn.setVelocity(0); //setVelocity actively holds the motor in it's current position
                rightTurn.setVelocity(0);
                continue;
            }

        }

    }

    private double closestAngle(double target, double current) {
        double dir = (target % 360) - (current % 360);
        if (Math.abs(dir) > 180.0) {
            dir = -(Math.signum(dir) * 360.0) + dir;
        }
        return dir;
    }

}
