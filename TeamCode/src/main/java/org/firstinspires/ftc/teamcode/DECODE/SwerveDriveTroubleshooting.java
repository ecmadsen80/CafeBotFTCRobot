package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import java.util.ArrayList;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp(name="SwerveDrive Troubleshooting", group="Linear OpMode")

public class SwerveDriveTroubleshooting extends LinearOpMode {

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
        rightTurn.setDirection(DcMotor.Direction.REVERSE);


        leftTurn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightTurn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftTurn.setTargetPosition(0);
        rightTurn.setTargetPosition(0);

        leftTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        com.acmerobotics.dashboard.FtcDashboard dashboard = com.acmerobotics.dashboard.FtcDashboard.getInstance();
        com.acmerobotics.dashboard.telemetry.TelemetryPacket packet = new com.acmerobotics.dashboard.telemetry.TelemetryPacket();
        // Get the voltage sensor from the hardware map
        com.qualcomm.robotcore.hardware.VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();


        waitForStart();



        runtime.reset();
        //leftTurn.setPower(1.0);
        //rightTurn.setPower(1.0);
        int position= 0;
        double targetAngle = 0;

        while (opModeIsActive()) {
            double leftDrivePower = gamepad1.left_stick_y;
            double rightDrivePower = gamepad1.right_stick_y;
            //double leftTurnPower = gamepad1.left_trigger;
            //double rightTurnPower = gamepad1.right_trigger;
            leftTurn.setTargetPosition(0);
            rightTurn.setTargetPosition(0);
            leftTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftTurn.setVelocity(TURN_VELOCITY);
            rightTurn.setVelocity(TURN_VELOCITY);
            leftDrive.setPower(leftDrivePower);
            rightDrive.setPower(rightDrivePower);


            // 2. Get current values in Amps (Requires DcMotorEx)
            double currentLeftDrive  = leftDrive.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
            double currentRightDrive = rightDrive.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
            double currentLeftTurn   = leftTurn.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
            double currentRightTurn  = rightTurn.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS);
            // 1. Read the current voltage
            double voltage = batteryVoltageSensor.getVoltage();

            // 2. Add data to the packet for graphing
            packet.put("Battery Voltage", voltage);
            packet.put("Current/Left Drive (A)", currentLeftDrive);
            packet.put("Current/Right Drive (A)", currentRightDrive);
            packet.put("Current/Left Turn (A)", currentLeftTurn);
            packet.put("Current/Right Turn (A)", currentRightTurn);

            // 4. Send the packet to the dashboard
            dashboard.sendTelemetryPacket(packet);

            // 5. Also add to standard telemetry for the Driver Station phone
            telemetry.addData("Left Drive Current", "%.2f A", currentLeftDrive);
            telemetry.addData("Right Drive Current", "%.2f A", currentRightDrive);



            // Telemetry
            /*
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

             */

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
