package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.List;

@TeleOp(name="Turret test", group="Linear OpMode")
public class TurretMechanismOpmode extends OpMode {

    private Limelight3A limelight;
    private TouchSensor button;
    private TurretMechanism turret = new TurretMechanism();

    private DcMotorEx flywheel = null;
    private DcMotor primaryIntake;
    private DcMotor secondaryIntake;
    private Servo gate;
    private Servo light;
    private static final double TICKS_PER_REV = 28.0;
    private double targetRPM = 0;
    private boolean gateOpen = false;
    private PIDFCoefficients pidfFullWeight = new PIDFCoefficients(125,0.0,0.0,11);

    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    int stepIndex = 2;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Webcam 1");
        button = hardwareMap.get(TouchSensor.class, "button");

        limelight.pipelineSwitch(0);
        limelight.start();

        turret.init(hardwareMap);

        telemetry.addLine("Post-Init");

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        primaryIntake = hardwareMap.get(DcMotor.class, "PrimaryIntake");
        secondaryIntake = hardwareMap.get(DcMotor.class, "SecondaryIntake");
        light = hardwareMap.get(Servo.class, "light");+
        gate = hardwareMap.get(Servo.class, "gate");

        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfFullWeight);
        light.setPosition(0.0);
    }

    public void start() {
        turret.resetTimer();
    }

    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        turret.update(result);

        if (result != null && result.isValid()) {
            telemetry.addData("Limelight", "sees tag");
        }
        else {
            telemetry.addData("Limelight", "no tag");
        }

        double intakePower = gamepad1.right_trigger-gamepad1.left_trigger;
        if (intakePower > 0.1) {
            primaryIntake.setPower(0.8);
            secondaryIntake.setPower(-1);
        }
        else if (intakePower < -0.1) {
            primaryIntake.setPower(-0.8);
            secondaryIntake.setPower(1);
        }
        else {
            //Rest Powers to 0 when buttons not pushed
            primaryIntake.setPower(0.0);
            secondaryIntake.setPower(0.0);

        }


        if(gamepad1.xWasPressed()) {
            gateOpen = !gateOpen;
            gate.setPosition(gateOpen ? 1.0 : 0.0);
            light.setPosition(gateOpen ? 0.283 : 0.5);
        }

        if (gamepad1.dpadUpWasPressed()) {
            targetRPM = 1000;
        }

        if (gamepad1.dpadDownWasPressed()) {
            targetRPM = 0;

        }
        if (gamepad1.dpadRightWasPressed()) {
            targetRPM += 25;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            targetRPM -= 25;
        }

        double targetTPS = ((targetRPM) / 60.0) * TICKS_PER_REV;
        flywheel.setVelocity(targetTPS);

        telemetry.addData("kp", "%.5f", turret.getKP());
        telemetry.addData("kd", "%.5f", turret.getKD());
        telemetry.addData("Gate Open", gateOpen);
        telemetry.addData("Flywheel RPM", (flywheel.getVelocity() / TICKS_PER_REV) * 60);
        telemetry.addData("Flywheel Target RPM", targetRPM);

        telemetry.update();
    }
}
