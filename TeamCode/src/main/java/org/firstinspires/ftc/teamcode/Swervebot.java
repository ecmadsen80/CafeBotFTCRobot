package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.util.Range;

@TeleOp(group="BIOBUZZ", name="BIOBUZZ-Swerve")
public class Swervebot extends LinearOpMode {

    private DcMotorEx leftDrive = null;
    private DcMotorEx leftTurn = null;
    private DcMotorEx rightDrive = null;
    private DcMotorEx rightTurn = null;

    static final double GOBILDA_ENCODER_TICKS = 751.8;
    static final double TURN_VELOCITY = 2500; //i set to 1000 to move slower
    static final double DEADZONE = 0.1;

    private I2cDeviceSynch as5600Left;
    private I2cDeviceSynch as5600Right;

    private Intake intake = new Intake();

    private static final int AS5600_ADDR = 0x36;
    private static final int ANGLE_REGISTER = 0x0E;

    private static final double LEFT_ZERO_POSITION = 173.32;
    private static final double RIGHT_ZERO_POSITION = 354.375;

    private boolean speedToggle = false;

    public void runOpMode() {
        //Device setup code
        leftDrive  = hardwareMap.get(DcMotorEx.class, "left_drive");
        leftTurn   = hardwareMap.get(DcMotorEx.class, "left_turn");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        rightTurn  = hardwareMap.get(DcMotorEx.class, "right_turn");
        intake.init(hardwareMap);



        //Absolute encoder setup code
        Rev2mDistanceSensor dummySensorL =
                hardwareMap.get(Rev2mDistanceSensor.class, "as5600Left");
        Rev2mDistanceSensor dummySensorR =
                hardwareMap.get(Rev2mDistanceSensor.class, "as5600Right");

        as5600Left = dummySensorL.getDeviceClient();
        as5600Left.setI2cAddress(I2cAddr.create7bit(AS5600_ADDR));
        as5600Left.engage();

        as5600Right = dummySensorR.getDeviceClient();
        as5600Right.setI2cAddress(I2cAddr.create7bit(AS5600_ADDR));
        as5600Right.engage();

        //Motor Parameter Setup
        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftTurn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightTurn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftTurn.setTargetPosition(0);
        rightTurn.setTargetPosition(0);

        leftTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        //feederLever.setPosition(0); //down

        //dashboard initialization



        //IMU initialization


        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();



        waitForStart();

        leftTurn.setPower(1.0);
        rightTurn.setPower(1.0);

        double leftStart = getAngle(as5600Left);
        double rightStart = getAngle(as5600Right);

        //Wheel alignment code
        double currentAngle = getAngle(as5600Left);
        double zeroAngle = LEFT_ZERO_POSITION;
        double angleError = ((zeroAngle - currentAngle + 540) % 360) - 180;

        int deltaTicks = (int) Math.round(angleError / 360.0 * GOBILDA_ENCODER_TICKS);

        leftTurn.setTargetPosition(leftTurn.getCurrentPosition() - deltaTicks);

        leftTurn.setVelocity(TURN_VELOCITY);
        currentAngle = (360- getAngle(as5600Right)) % 360;
        zeroAngle = RIGHT_ZERO_POSITION;
        angleError = ((zeroAngle - currentAngle + 540) % 360) - 180;

        deltaTicks = (int) Math.round(angleError / 360.0 * GOBILDA_ENCODER_TICKS);
        rightTurn.setTargetPosition(
                rightTurn.getCurrentPosition() - deltaTicks
        );
        rightTurn.setVelocity(TURN_VELOCITY);

        while (leftTurn.isBusy() || rightTurn.isBusy()) {
            telemetry.addLine("Aligning wheels...");
            telemetry.addData("left start", leftStart);
            telemetry.addData("rightt start", rightStart);
            telemetry.addData("left now", getAngle(as5600Left));
            telemetry.addData("right now", getAngle(as5600Right));

            telemetry.update();
        }

        leftTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftTurn.setTargetPosition(0);
        rightTurn.setTargetPosition(0);
        leftTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        while (opModeIsActive()) {
            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double turn = 0;

            // 2. Define Rotation Vectors
            // For a 2-module robot to spin, one wheel points "up/left" and the other "up/right"
            // We'll assume the modules are on the left and right sides.
            double leftRotationX = 0;   // Vertical component of rotation for left pod
            double leftRotationY = turn;
            double rightRotationX = 0;
            double rightRotationY = -turn;

            // 3. Combine Translation (Left Stick) and Rotation (Right Stick)
            double combinedLeftX = lx + leftRotationX;
            double combinedLeftY = ly + leftRotationY;
            double combinedRightX = lx + rightRotationX;
            double combinedRightY = ly + rightRotationY;

            // 4. Calculate Individual Magnitudes and Angles
            double magLeft = Math.hypot(combinedLeftX, combinedLeftY);
            double magRight = Math.hypot(combinedRightX, combinedRightY);

            double targetAngleLeft = Math.toDegrees(Math.atan2(combinedLeftX, combinedLeftY));
            double targetAngleRight = Math.toDegrees(Math.atan2(combinedRightX, combinedRightY));

            if (targetAngleLeft < 0) targetAngleLeft += 360;
            if (targetAngleRight < 0) targetAngleRight += 360;
            //get current encoder positions
            int leftCurrentTicks = leftTurn.getCurrentPosition();
            int rightCurrentTicks = rightTurn.getCurrentPosition();
            // Convert to degrees
            int leftCurrentDegrees = (int)((leftCurrentTicks / (double)GOBILDA_ENCODER_TICKS) * 360);
            int rightCurrentDegrees = (int)((rightCurrentTicks / (double)GOBILDA_ENCODER_TICKS) * 360);

            // 5. Optimization for LEFT Pod
            double driveMultLeft = 1.0;
            double diffLeft = closestAngle(targetAngleLeft, leftCurrentDegrees); //here closest angle for turning the wheel is calculated
            if (abs(diffLeft) > 90) {                                       //if the closest angle is > 90 deg
                driveMultLeft = -1.0;                                            //reverses the direction of the motor using "driveMultLeft"
                targetAngleLeft = (targetAngleLeft + 180) % 360;                 //adjusts the angle to the opposite angle
            }

            // 6. Optimization for RIGHT Pod
            double driveMultRight = 1.0;
            double diffRight = closestAngle(targetAngleRight, rightCurrentDegrees);
            if (abs(diffRight) > 90) {
                driveMultRight = -1.0;
                targetAngleRight = (targetAngleRight + 180) % 360;
            }

            // 7. Calculate Final Motor Targets
            double moveLeft = closestAngle(targetAngleLeft, leftCurrentDegrees) + leftCurrentDegrees;
            double moveRight = closestAngle(targetAngleRight, rightCurrentDegrees) + rightCurrentDegrees;

            leftTurn.setTargetPosition((int)((moveLeft / 360.0) * GOBILDA_ENCODER_TICKS));
            rightTurn.setTargetPosition((int)((moveRight / 360.0) * GOBILDA_ENCODER_TICKS));

            leftTurn.setVelocity(TURN_VELOCITY);
            rightTurn.setVelocity(TURN_VELOCITY);


            // 8. Apply Final Drive Power
            // Normalize magnitudes if they exceed 1.0
            double maxMag = Math.max(1.0, Math.max(magLeft, magRight));


            // Switch between fast speed and slow speed
            if (gamepad1.yWasPressed()) {
                speedToggle = !speedToggle; // Switches true to false or false to true
            }



            // Determine the speed multiplier based on the toggle state
            double speedLimit = speedToggle ? 0.5 : 1.0;

            // Motor power combination of left and right sides and fast/slow state
            leftDrive.setPower(Range.clip((magLeft / maxMag) * driveMultLeft * speedLimit, -speedLimit, speedLimit));
            rightDrive.setPower(Range.clip((magRight / maxMag) * driveMultRight * speedLimit, -speedLimit, speedLimit));
        }

    }
    private double getAngle(I2cDeviceSynch as5600) {
        byte[] angleBytes = as5600.read(ANGLE_REGISTER, 2);

        int high = angleBytes[0] & 0xFF;
        int low = angleBytes[1] & 0xFF;

        int rawAngle = ((high << 8) | low) & 0x0FFF;
        return rawAngle * 360.0 / 4096.0;
    }
    private double closestAngle(double target, double current) { //method inputs current and target encoder position in degrees
        double dir = (target % 360) - (current % 360);           //reduces encoder angle to a value between 0 and 359
        if (abs(dir) > 180.0) {                             //If the angle is > 180 degrees
            dir = -(Math.signum(dir) * 360.0) + dir;            //subtracts the angle from 360
        }
        return dir;
    }
}