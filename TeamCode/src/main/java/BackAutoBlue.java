package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto (Back, Blue)", group="Linear OpMode")
public class BackAutoBlue extends LinearOpMode {

    // ==========================
    // HARDWARE
    // ==========================
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    private DcMotorEx leftTurn;
    private DcMotorEx rightTurn;

    private DcMotorEx flywheel;
    private DcMotor intake;
    private DcMotor pusher;
    private Limelight3A limelight;
    private DcMotor pusher1;

    private Servo feederLever;
    private Servo light;

    private ElapsedTime runtime = new ElapsedTime();

    // ==========================
    // EASY-TO-TUNE TIMES (SECONDS)
    // ==========================
    private static final double BACK_UP_TIME = 3.5;
    private static final double TURN_TIME    = 0.35;

    // ==========================
    // AUTONOMOUS STATES
    // ==========================
    private enum AutoState {
        BACK_UP,
        TURN,
        SHOOT,
        DONE
    }

    private AutoState state = AutoState.BACK_UP;

    @Override
    public void runOpMode() {

        // ==========================
        // MAP HARDWARE
        // ==========================
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftTurn  = hardwareMap.get(DcMotorEx.class, "left_turn");
        rightTurn = hardwareMap.get(DcMotorEx.class, "right_turn");

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        intake   = hardwareMap.get(DcMotor.class, "intake");
        pusher   = hardwareMap.get(DcMotor.class, "rightpusher");
        pusher1  = hardwareMap.get(DcMotor.class, "leftpusher");
        limelight = hardwareMap.get(Limelight3A.class, "Webcam 1");

        feederLever = hardwareMap.get(Servo.class, "feederLever");
        light = hardwareMap.get(Servo.class, "light");

        // ==========================
        // DRIVE SETUP
        // ==========================
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // ==========================
        // TURN MOTOR SETUP (UNCHANGED)
        // ==========================
        leftTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftTurn.setTargetPosition(0);
        rightTurn.setTargetPosition(0);

        leftTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftTurn.setPower(1.0);
        rightTurn.setPower(1.0);

        // ==========================
        // SHOOTER SETUP
        // ==========================
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        limelight.pipelineSwitch(0);
        limelight.start();
        feederLever.setPosition(0.0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        double distance = 0;
        double targetRPM = 0;

        waitForStart();
        runtime.reset();

        // ==========================
        // MAIN LOOP
        // ==========================
        while (opModeIsActive()) {

            switch (state) {

                // --------------------------
                case BACK_UP:
                    leftDrive.setPower(0.5);
                    rightDrive.setPower(-0.5);

                    if (runtime.seconds() > BACK_UP_TIME) {
                        leftDrive.setPower(0);
                        rightDrive.setPower(0);
                        runtime.reset();
                        state = AutoState.TURN;
                    }
                    break;

                // --------------------------
                case TURN:
                    leftDrive.setPower(-0.30);
                    rightDrive.setPower(-0.30);
                    LLResult result = limelight.getLatestResult();

                    telemetry.addLine(Boolean.toString(result.isValid()));
                    if (result.isValid()) {

                        if (result.getTx() < 0) {
                            leftDrive.setPower(0);
                            rightDrive.setPower(0);
                            distance = Math.pow((result.getTa()/9946.27),-0.560091);
                            targetRPM = 3238.403 + (2206.559 - 3238.403) / (1 + (Math.pow((distance / 141.1671), 3.98712)));
                            runtime.reset();
                            state = AutoState.SHOOT;
                        }
                    }
                    break;

                // --------------------------
                case SHOOT:

                    flywheel.setVelocity(targetRPM);
                    sleepSeconds(2);
                    SHOOTHEBALLS();
                    SHOOTHEBALLS();

                    SHOOTHEBALLS();
                    state = AutoState.DONE;
                    break;

                // --------------------------
                case DONE:
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    flywheel.setPower(0);
                    intake.setPower(0);
                    pusher.setPower(0);
                    pusher1.setPower(0);
                    // Idle safely
                    break;
            }

            telemetry.addData("State", state);
            telemetry.update();
        }
        limelight.stop();
    }

    // ==========================
    // SHOOTER SEQUENCE
    // ==========================
    private void SHOOTHEBALLS() {
        sleepSeconds(1);
        feederLever.setPosition(1.0);
        sleepSeconds(1);
        feederLever.setPosition(0.0);

        intake.setPower(-1.0);
        pusher.setPower(-1.0);
        pusher1.setPower(-1.0);

        sleepSeconds(1);

        intake.setPower(0);
        pusher.setPower(0);
        pusher1.setPower(0);
    }

    private void sleepSeconds(int seconds) {
        try {
            Thread.sleep(seconds * 1000L);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
