package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Disabled
@TeleOp(name="Motor RPM Control", group="Concept")
public class sanitycheck extends LinearOpMode {

    private DcMotorEx myMotor = null;
    private Servo feederLever = null;

    // Define your motor's counts per revolution
    static final double COUNTS_PER_MOTOR_REV = 28; // Example value

    @Override
    public void runOpMode() {

        // Initialize the motor as DcMotorEx
        myMotor = hardwareMap.get(DcMotorEx.class, "flywheel");
        feederLever = hardwareMap.get(Servo.class, "feederLever");
        feederLever.setPosition(0.0);

        // Set the motor direction if needed
        myMotor.setDirection(DcMotor.Direction.REVERSE);

        // Set the run mode to RUN_USING_ENCODER
        myMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Desired RPM
        double targetRPM = 0;

        // Convert RPM to Ticks Per Second (TPS)
        double targetTPS = (targetRPM / 60) * COUNTS_PER_MOTOR_REV;

        // Set the motor velocity
        myMotor.setVelocity(targetTPS);

        while (opModeIsActive()) {
            if (feederLever != null){
                if (gamepad1.dpad_up) {
                    feederLever.setPosition(0.0);
                }

                else if (gamepad1.dpad_down){
                    feederLever.setPosition(1.0);
                }
            }
            telemetry.addData("Current Velocity (TPS)", myMotor.getVelocity());
            telemetry.addData("Target Velocity (TPS)", targetTPS);
            telemetry.update();
        }

        // Stop the motor when OpMode ends
        myMotor.setPower(0);
    }
}