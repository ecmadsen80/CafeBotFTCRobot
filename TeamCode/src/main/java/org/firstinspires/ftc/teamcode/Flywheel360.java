package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Flywheel360")
public class Flywheel360 extends LinearOpMode {

    private static final double MAX_RPM = 3000.0;
    private static final double MIN_RPM = -3000.0;

    private static final double STEP = 500.0;
    private static final double TICKS_PER_REV = 28.0;

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "motor1");

        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        double targetRPM = 500;

        waitForStart();

        while (opModeIsActive()) {

            // --- Step Up ---
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

            telemetry.addData("Target RPM", targetRPM);
            telemetry.addData("TPS", targetTPS);
            telemetry.addData("Actual TPS", flywheel.getVelocity());
            telemetry.update();
        }
    }
}
