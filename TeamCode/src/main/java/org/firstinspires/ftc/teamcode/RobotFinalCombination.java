package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.ArrayList;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name="Final TeleOp Robot Program", group="Linear OpMode")

public class RobotFinalCombination extends LinearOpMode {

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
    private Limelight3A limelight;
    private Servo feederLever = null;
    //private DigitalChannel laserInput;

    private static final double MAX_RPM = 5000;
    private static final double MIN_RPM = 0;

    private static final double STEP = 500.0;
    private static final double TICKS_PER_REV = 28.0;



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
    private double targetRPM = 1500;

    @Override
    public void runOpMode() {

        //Initialization
        leftDrive  = hardwareMap.get(DcMotorEx.class, "left_drive");
        leftTurn   = hardwareMap.get(DcMotorEx.class, "left_turn");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        rightTurn  = hardwareMap.get(DcMotorEx.class, "right_turn");
        intake     = hardwareMap.get(DcMotor.class, "intake");
        leftPusher = hardwareMap.get(DcMotor.class, "leftpusher");
        rightPusher= hardwareMap.get(DcMotor.class, "rightpusher");
        light = hardwareMap.get(Servo.class, "light");
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        feederLever = hardwareMap.get(Servo.class, "feederLever"); //0 is down, 1 is up
        //laserInput = hardwareMap.get(DigitalChannel.class, "distancer");
        light  = hardwareMap.get(Servo.class, "light");
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

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

        feederLever.setPosition(0); //down

        //dashboard initialization
        com.acmerobotics.dashboard.FtcDashboard dashboard = com.acmerobotics.dashboard.FtcDashboard.getInstance();
        com.acmerobotics.dashboard.telemetry.TelemetryPacket packet = new com.acmerobotics.dashboard.telemetry.TelemetryPacket();
        com.qualcomm.robotcore.hardware.VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();


        waitForStart();



        runtime.reset();
        leftTurn.setPower(1.0); //I'm not sure why this is here
        rightTurn.setPower(1.0); //Ditto

        int position= 0;
        double targetAngle = 0;

        while (opModeIsActive()) {


            //Swerve Drive
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y; // FTC Y inverted
            double turn = gamepad1.right_stick_x;

            // Raw magnitude
            double rawMag = Math.hypot(x, y);
            // Normalize stick vector
            double nx = x / rawMag;
            double ny = y / rawMag;

            // Scaled magnitude (optional throttle curve)
            double mag = Range.clip(rawMag, 0, 1);

            // Drive power calculation with Turn (Z-axis rotation). This doesn't work great if wheels are
            // perpendicular to the robot
            double leftPower = mag + turn;
            double rightPower = mag - turn;

            // Use Range.clip to ensure power stays between -1 and 1; Removed when added new swerve optimization
            //leftDrive.setPower(Range.clip(leftPower, -1, 1));
            //rightDrive.setPower(Range.clip(rightPower, -1, 1));

            // Calculate targetAngle in degrees [0,360) from the joystick input.
            double targetAngleDeg = Math.toDegrees(Math.atan2(nx, ny));

            //Normalize to 360
            if (targetAngleDeg < 0) targetAngleDeg += 360; //0 is directly right (1, 0), 90 is directly up (0, 1), 180 is directly left (-1, 0),             // and 270 is directly down (-1,0)


            //get current encoder positions
            int leftCurrentTicks = leftTurn.getCurrentPosition();
            int rightCurrentTicks = rightTurn.getCurrentPosition();
            // Convert to degrees
            int leftCurrentDegrees = (int)((leftCurrentTicks / (double)TURN_TICKS_PER_REV) * 360);
            int rightCurrentDegrees = (int)((rightCurrentTicks / (double)TURN_TICKS_PER_REV) * 360);


            //calculates the closest angle to the target angle and the sets the move to angle by adding to current degrees
            //removed this when added the below "optimization"
            //double moveToAngleLeft = closestAngle(targetAngleDeg, leftCurrentDegrees) + leftCurrentDegrees;
            //double moveToAngleRight = closestAngle(targetAngleDeg, rightCurrentDegrees) + rightCurrentDegrees;

            // --- SWERVE OPTIMIZATION --- Only calculates based on one side? Will this be a problem?
            double driveMultiplier = 1.0;
            double diffBetweenAngles = closestAngle(targetAngleDeg, leftCurrentDegrees);

            // If the required turn is more than 90 degrees, reverse the motor
            // and turn to the opposite angle instead.
            if (Math.abs(diffBetweenAngles) > 90) {
                driveMultiplier = -1.0;
                // Add 180 to the target and keep it in 0-360 range
                targetAngleDeg = (targetAngleDeg + 180) % 360;
            }

            // Re-calculate the moveToAngle based on the potentially optimized targetAngleDeg
            double optimizedMoveToAngleLeft = closestAngle(targetAngleDeg, leftCurrentDegrees) + leftCurrentDegrees;
            double optimizedMoveToAngleRight = closestAngle(targetAngleDeg, rightCurrentDegrees) + rightCurrentDegrees;

            // Correct degrees → encoder ticks
            int targetTicksRight = (int)((optimizedMoveToAngleRight / 360.0) * TURN_TICKS_PER_REV);
            int targetTicksLeft = (int)((optimizedMoveToAngleLeft / 360.0) * TURN_TICKS_PER_REV);

            // Turn motors
            leftTurn.setTargetPosition(targetTicksLeft);
            rightTurn.setTargetPosition(targetTicksRight);

            //using setVelocity should use the internal PID controller
            leftTurn.setVelocity(TURN_VELOCITY);
            rightTurn.setVelocity(TURN_VELOCITY);

            // Update the drive power to account for potential reversal
            leftDrive.setPower(Range.clip(leftPower * driveMultiplier, -1, 1));
            rightDrive.setPower(Range.clip(rightPower * driveMultiplier, -1, 1));




            //Intake
            double intakePower = gamepad1.left_trigger-gamepad1.right_trigger;
            if (intakePower > 0.1) {
                intake.setPower(-1.0);
                leftPusher.setPower(-1.0);
                rightPusher.setPower(-1.0);
            }
            if (intakePower < -0.1) {
                intake.setPower(1.0);
            }


            //Flywheel - right now manually adjusted
            if (gamepad1.dpad_right) {
                targetRPM += STEP;
                if (targetRPM > MAX_RPM) targetRPM = MAX_RPM;

                // simple debounce
                sleep(200);
            }
            // --- Step Down ---
            if (gamepad1.dpad_left) {
                targetRPM -= STEP;
                if (targetRPM < MIN_RPM) targetRPM = MIN_RPM;
                sleep(200);
            }
            // Convert RPM → ticks per second
            double targetTPS = (targetRPM / 60.0) * TICKS_PER_REV;

            // Apply velocity
            flywheel.setVelocity(targetTPS);



            //Fire Ball
            if (gamepad1.aWasPressed()) {
                shootTheBall();
            }
            //Rest Powers to 0 when buttons not pushed
            intake.setPower(0.0);
            leftPusher.setPower(0);
            rightPusher.setPower(0);
            // Deadzone
            if (rawMag < DEADZONE) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                leftTurn.setVelocity(0); //setVelocity actively holds the motor in it's current position
                rightTurn.setVelocity(0);
            }


            //Dashboard Graphing
            double currentLeftDrive  = leftDrive.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
            double currentRightDrive = rightDrive.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
            double currentLeftTurn   = leftTurn.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
            double currentRightTurn  = rightTurn.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);

            double voltage = batteryVoltageSensor.getVoltage();

            packet.put("Battery Voltage", voltage);
            packet.put("Current/Left Drive (A)", currentLeftDrive);
            packet.put("Current/Right Drive (A)", currentRightDrive);
            packet.put("Current/Left Turn (A)", currentLeftTurn);
            packet.put("Current/Right Turn (A)", currentRightTurn);

            dashboard.sendTelemetryPacket(packet);




            // Telemetry
            telemetry.addData("IntakePower", intakePower);
            telemetry.addData("Target Angle deg", targetAngleDeg);
            telemetry.addData("flywheel target RPM", targetRPM);
            telemetry.addData("Left Target Position", optimizedMoveToAngleLeft);
            telemetry.addData("Right Target Position", optimizedMoveToAngleRight);
            telemetry.addData("Left Current Position", leftCurrentDegrees);
            telemetry.addData("Right Current Position", rightCurrentDegrees);
            //telemetry.addData("Raw mag", rawMag);
            //telemetry.addData("Mag", mag);
            //telemetry.addData("XY", "%.2f %.2f", x, y);
            telemetry.update();



        }

    }

    private double closestAngle(double target, double current) {
        double dir = (target % 360) - (current % 360);
        if (Math.abs(dir) > 180.0) {
            dir = -(Math.signum(dir) * 360.0) + dir;
        }
        return dir;
    }

    public static void sleepSeconds(int seconds) {
        try {
            Thread.sleep(seconds * 1000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
    private void shootTheBall() {
        feederLever.setPosition(1.0); //fires one ball
        sleep(1000);
        feederLever.setPosition(0.0);

    }

}
