package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.LLResult;
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
import com.seattlesolvers.solverslib.controller.PIDFController;



@TeleOp(name="A Working TeleOp", group="Linear OpMode")

public class RobotFinalCombination extends LinearOpMode {

    // Declare OpMode members.

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftDrive = null;
    private DcMotorEx leftTurn = null;
    private DcMotorEx rightDrive = null;
    private DcMotorEx rightTurn = null;
    DcMotorEx flywheel = null;
    private DcMotor leftPusher;
    private DcMotor rightPusher;
    private DcMotor intake;
    private Servo light;
    private Limelight3A limelight;
    private Servo feederLever = null;
    //private DigitalChannel laserInput;

    //PID Controller for Aiming
    private PIDFController aimPid = new PIDFController(0.025, 0.0, 0.0, 0.01);

    private static final double MAX_RPM = 5000;
    private static final double MIN_RPM = 0;

    private static final double STEP = 50.0;
    private static final double TICKS_PER_REV = 28.0;

    private static final double RIGHT_STICK_ADJUSTER = 0.7;



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
    private double targetRPM = 0;
    private boolean xToggle = false; //to toggle between fast and slow motor speeds, false = slow
    private double turn = 0;
    private double distance = 0;
    // Timer to track how long the flywheel RPM has been stable
    private ElapsedTime rpmStableTimer = new ElapsedTime();

    // States for the Aim-and-Shoot subroutine
    private enum AimState {
        IDLE,       // Doing nothing
        AIMING,     // Robot is turning to face the tag
        SPINNING_UP,
        SHOOTING    // Robot is aimed, now shooting
    }    // Variable to track the current state

    private AimState currentAimState = AimState.IDLE;


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
        limelight = hardwareMap.get(Limelight3A.class, "Webcam 1");
        light = hardwareMap.get(Servo.class, "light");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
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

        //Limelight initialization
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();


        waitForStart();



        runtime.reset();
        leftTurn.setPower(1.0); //I'm not sure why this is here
        rightTurn.setPower(1.0); //Ditto

        int position= 0;
        double targetAngle = 0;


        while (opModeIsActive()) {

            //LimeLight

            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                // distance is calculated based on linear regression with the equation Ta = distance
                double distance = Math.pow((result.getTa()/9946.27),-0.560091);

            } else {
                telemetry.addData("Limelight", "No data available");
            }

            //New Swerve Drive Logic to incorporate Turning while strafing
            // 1. Capture Joystick Inputs
            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double turn = 0;




            // --- Aim and Shoot State Machine allows ball to be loaded and fired with one button push
            switch (currentAimState) {
                case IDLE:
                    // If 'A' is pressed, start the aiming process
                    if (gamepad1.aWasPressed()) {
                        currentAimState = AimState.AIMING;
                    }
                    break;

                case AIMING:
                    // Use the auto-aim logic to turn the robot
                    turn = pointAtTag();

                    // Check if we are successfully aimed at the target
                    if (result.isValid() && Math.abs(result.getTx()) < 5.0) { // Aim is within 2 degrees
                        // If aimed, move to the next state and shoot
                        rpmStableTimer.reset();
                        currentAimState = AimState.SPINNING_UP;

                    }

                    // If 'A' is pressed again, cancel the routine
                    if (gamepad1.aWasPressed()) {
                        currentAimState = AimState.IDLE;
                    }
                    break;

                case SPINNING_UP:
                    // In this state, we are aimed, but waiting for the flywheel to be stable.
                    // Keep the robot aimed at the tag in case it drifts.
                    turn = pointAtTag();

                    // Get the current flywheel velocity in RPM
                    double currentRPM = flywheel.getVelocity() / TICKS_PER_REV * 60;

                    // Check if the flywheel is within the desired speed range (e.g., 95% of target)
                    if (targetRPM > 0 && Math.abs((currentRPM-targetRPM)/targetRPM) < 0.05) {
                        // If the speed has been stable for 500ms, move to the SHOOTING state.
                        if (rpmStableTimer.milliseconds() >= 500) {
                            currentAimState = AimState.SHOOTING;
                        }
                    } else {
                        // If the speed drops out of range, reset the stability timer.
                        rpmStableTimer.reset();
                    }

                    // Allow the driver to cancel
                    if (gamepad1.aWasPressed()) {
                        flywheel.setVelocity(0);
                        currentAimState = AimState.IDLE;
                    }
                    break;

                case SHOOTING:
                    shootTheBall(); // This will move the servo
                    currentAimState = AimState.IDLE;
                    break;
            }




            //Aiming the Robot if Right Bumper Pushed, otherwise use the right stick
            //Aiming the Robot if Right Bumper Pushed, otherwise use the right stick
            // Make sure the state machine is not running before allowing manual control
            if (currentAimState == AimState.IDLE) {
                if (gamepad1.right_bumper) {
                    turn = pointAtTag();
                } else {
                    turn = gamepad1.right_stick_x * RIGHT_STICK_ADJUSTER;
                    aimPid.reset(); // Reset PID memory when not in use
                }
            }

            //Aim and Fire Ball


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
            int leftCurrentDegrees = (int)((leftCurrentTicks / (double)TURN_TICKS_PER_REV) * 360);
            int rightCurrentDegrees = (int)((rightCurrentTicks / (double)TURN_TICKS_PER_REV) * 360);

            // 5. Optimization for LEFT Pod
            double driveMultLeft = 1.0;
            double diffLeft = closestAngle(targetAngleLeft, leftCurrentDegrees);
            if (Math.abs(diffLeft) > 90) {
                driveMultLeft = -1.0;
                targetAngleLeft = (targetAngleLeft + 180) % 360;
            }

            // 6. Optimization for RIGHT Pod
            double driveMultRight = 1.0;
            double diffRight = closestAngle(targetAngleRight, rightCurrentDegrees);
            if (Math.abs(diffRight) > 90) {
                driveMultRight = -1.0;
                targetAngleRight = (targetAngleRight + 180) % 360;
            }

            // 7. Calculate Final Motor Targets
            double moveLeft = closestAngle(targetAngleLeft, leftCurrentDegrees) + leftCurrentDegrees;
            double moveRight = closestAngle(targetAngleRight, rightCurrentDegrees) + rightCurrentDegrees;

            leftTurn.setTargetPosition((int)((moveLeft / 360.0) * TURN_TICKS_PER_REV));
            rightTurn.setTargetPosition((int)((moveRight / 360.0) * TURN_TICKS_PER_REV));

            leftTurn.setVelocity(TURN_VELOCITY);
            rightTurn.setVelocity(TURN_VELOCITY);

            // 8. Apply Final Drive Power
            // Normalize magnitudes if they exceed 1.0
            double maxMag = Math.max(1.0, Math.max(magLeft, magRight));


            // Switch between fast speed and slow speed
            if (gamepad1.xWasPressed()) {
                xToggle = !xToggle; // Switches true to false or false to true
            }

            // Determine the speed multiplier based on the toggle state
            double speedLimit = xToggle ? 0.5 : 1.0;

            // Motor power combination of left and right sides and fast/slow state
            leftDrive.setPower(Range.clip((magLeft / maxMag) * driveMultLeft * speedLimit, -speedLimit, speedLimit));
            rightDrive.setPower(Range.clip((magRight / maxMag) * driveMultRight * speedLimit, -speedLimit, speedLimit));








            //Intake
            double intakePower = gamepad1.right_trigger-gamepad1.left_trigger;
            if (intakePower > 0.1) {
                intake.setPower(-1.0);
                leftPusher.setPower(-1.0);
                rightPusher.setPower(-1.0);
            }
            else if (intakePower < -0.1) {
                intake.setPower(1.0);
                leftPusher.setPower(0);
                rightPusher.setPower(0);
            }
            else {
                //Rest Powers to 0 when buttons not pushed
                intake.setPower(0.0);
                leftPusher.setPower(0);
                rightPusher.setPower(0);
            }



            //flywheel manual control
            if (gamepad1.dpadDownWasPressed()){
                targetRPM = 2000;
            }
            if (gamepad1.dpadUpWasPressed()){
                targetRPM = 0;
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

            //singlebutton shoot only
            if (gamepad1.bWasPressed()){
                shootTheBall();
            }



            // Deadzone
            if (magLeft < DEADZONE && magRight < DEADZONE) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                leftTurn.setVelocity(0); //setVelocity actively holds the motor in it's current position
                rightTurn.setVelocity(0);
            }


            if (gamepad1.leftBumperWasPressed()) {
                aimPid.setF(aimPid.getF()+ 0.005);
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
            telemetry.addData("kP", aimPid.getP());
            telemetry.addData("kF", aimPid.getF());
            telemetry.addData("IntakePower", intakePower);
            // Update these lines at the bottom of your loop
            telemetry.addData("flywheel RPM", targetRPM);
            telemetry.addData("Left Target Angle", targetAngleLeft);
            telemetry.addData("Right Target Angle", targetAngleRight);
            telemetry.addData("Left Move To", moveLeft);
            telemetry.addData("Right Move To", moveRight);
            telemetry.addData("Left Current Deg", leftCurrentDegrees);
            telemetry.addData("Right Current Deg", rightCurrentDegrees);
            telemetry.addData("distance:", "%.2f", distance);
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ta", result.getTa());


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

    private double pointAtTag() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // SolversLib uses calculate(target, current)
            // We want the horizontal offset (tx) to be 0
            double pidOutput = aimPid.calculate(0, result.getTx());
            return Range.clip(pidOutput, -1.0, 1.0);
        }

        aimPid.reset(); // Clear integral sum when target is lost
        return 0;
    }






}
