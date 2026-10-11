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

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

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
        primaryIntake = hardwareMap.get(DcMotor.class, "primaryIntake");
        secondaryIntake = hardwareMap.get(DcMotor.class, "secondaryIntake");
        light = hardwareMap.get(Servo.class, "light");
        gate = hardwareMap.get(Servo.class, "gate");

        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfFullWeight);
        light.setPosition(0.0);
    }

    public void start() {

    }
    private double distanceFromTag() {
        LLResult result = limelight.getLatestResult();
        double distance = 0;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults(); // Get the list of all visible tags
        for (LLResultTypes.FiducialResult fr : fiducials) {
            if (true) {
                Pose3D targetPose = fr.getRobotPoseTargetSpace();               //Gets the "pose" of the robot relative to the AprilTag
                double x = targetPose.getPosition().x;                          //Obtains x, y, and z of the robot relative to the AprilTag
                double y = targetPose.getPosition().y;
                double z = targetPose.getPosition().z;

                telemetry.addData("April Tag", "Found");
                distance = Math.sqrt(Math.pow(x, 2) + Math.pow(z, 2));
                telemetry.addData("distanceFromTag", distance);
                telemetry.addData("Formula Distance", Math.pow((result.getTa() / 9946.27), -0.560091));


            }
        }
        return distance;
    }
    @Override
    public void loop() {
        LLResult result = limelight.getLatestResult();

        turret.update(result);

        if (result != null && result.isValid()) {
            telemetry.addData("Limelight", "sees tag");
            telemetry.addData("Distance", "%.5f", distanceFromTag());
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

        if (gamepad1.aWasPressed()) {
            targetRPM = 2500;
        }


        if (gamepad1.dpadRightWasPressed()) {
            targetRPM += 25;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            targetRPM -= 25;
        }

        double targetTPS = ((targetRPM) / 60.0) * TICKS_PER_REV;
        flywheel.setVelocity(targetTPS);

        telemetry.addData("Gate Open", gateOpen);
        telemetry.addData("Flywheel RPM", (flywheel.getVelocity() / TICKS_PER_REV) * 60);
        telemetry.addData("Flywheel Target RPM", targetRPM);


        telemetry.update();
    }
}
