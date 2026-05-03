package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.TouchSensor;

import java.util.List;
@TeleOp(name="Turret test", group="Linear OpMode")
public class TurretMechanismOpmode extends OpMode {

    private Limelight3A limelight;
    private TouchSensor button;
    private TurretMechanism turret = new TurretMechanism();
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
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            turret.setKP(turret.getKP() - stepSizes[stepIndex]);
        }
        if (gamepad1.dpadRightWasPressed()) {
            turret.setKP(turret.getKP() + stepSizes[stepIndex]);
        }
        if (gamepad1.dpadDownWasPressed()) {
            turret.setKD(turret.getKD() - stepSizes[stepIndex]);
        }
        if (gamepad1.dpadUpWasPressed()) {
            turret.setKD(turret.getKD() + stepSizes[stepIndex]);
        }

        if (button.isPressed()) {
            telemetry.addLine("button :)");
            turret.resetTurret();
        }

        telemetry.addData("kp", "%.5f", turret.getKP());
        telemetry.addData("kd", "%.5f", turret.getKD());

        telemetry.update();

    }
}
