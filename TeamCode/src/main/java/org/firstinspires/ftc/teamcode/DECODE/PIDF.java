package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class
PIDF extends OpMode {
    public DcMotorEx flywheel;

    public double highvelocity =  2500;
    public double lowvel = 2000;
    public double curvel = highvelocity;

    double F=0;
    double P=0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepIndex = 1;

    com.acmerobotics.dashboard.FtcDashboard dashboard = com.acmerobotics.dashboard.FtcDashboard.getInstance();
    com.acmerobotics.dashboard.telemetry.TelemetryPacket packet = new com.acmerobotics.dashboard.telemetry.TelemetryPacket();
    //com.qualcomm.robotcore.hardware.VoltageSensor batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

    @Override
    public void init() {
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidf = new PIDFCoefficients(P,0,0,F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidf);
        telemetry.addLine("init");


    }

    @Override
    public void loop() {
        if (gamepad1.yWasPressed()) {
            if (curvel == highvelocity) {
                curvel = lowvel;
            } else {
                curvel = highvelocity;

            }


        }
        if (gamepad1.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if (gamepad1.dpadLeftWasPressed()) {
            F -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadRightWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if (gamepad1.dpadDownWasPressed()) {
            P -= stepSizes[stepIndex];
        }
        if (gamepad1.dpadUpWasPressed()) {
            P += stepSizes[stepIndex];
        }
        PIDFCoefficients pidf = new PIDFCoefficients(P,0,0,F);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidf);

        flywheel.setVelocity((curvel/60)*28);

        double curvel2 = flywheel.getVelocity();
        curvel2 = (curvel2/28)*60;
        double error = curvel - curvel2;
        telemetry.addData("Err", "%.2f", error);
        telemetry.addData("Target", curvel);
        telemetry.addData("Current", curvel2);
        telemetry.addData("P", "%.4f", P);
        telemetry.addData("F", "%.4f", F);
        telemetry.addData("Stepsize", "%.4f", stepSizes[stepIndex]);

        // --- Add Flywheel Data to Dashboard Packet ---
        packet.put("Flywheel Target Velocity", curvel);
        packet.put("Flywheel Current Velocity", curvel2);
        packet.put("Flywheel Error", error);
        // ---------------------------------------------
        //double voltage = batteryVoltageSensor.getVoltage();

        //packet.put("Battery Voltage", voltage);
        dashboard.sendTelemetryPacket(packet);
    }
}