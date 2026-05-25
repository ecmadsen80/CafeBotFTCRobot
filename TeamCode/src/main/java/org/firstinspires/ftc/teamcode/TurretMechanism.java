package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretMechanism {

    private DcMotorEx turret;

    // Limelight settings
    private double goalX = 0;
    private double angleTolerance = 3.5;

    // Motor settings
    private final double MAX_POWER = 0.8;

    // 26.9:1 Yellow Jacket (751.8 ticks/rev)
    // External ratio: 21T -> 70T
    private final double TICKS_PER_DEGREE = 6.96;

    // Optional turret limits
    private final int MIN_POSITION = -3000;
    private final int MAX_POSITION = 3000;

    private boolean isResetting = false;

    public void init(HardwareMap hwMap) {

        turret = hwMap.get(DcMotorEx.class, "turret");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        turret.setTargetPosition(0);

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        turret.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        turret.setPower(MAX_POWER);
    }

    public void resetTurret() {

        isResetting = true;

        turret.setTargetPosition(0);
        turret.setPower(0.6);
    }

    public void update(LLResult result) {

        // Reset mode
        if (isResetting) {

            if (!turret.isBusy()) {
                isResetting = false;
            }

            return;
        }

        // No target found
        if (result == null || !result.isValid()) {
            return;
        }

        // Horizontal offset from target
        double error = goalX - result.getTx();

        // Within tolerance → don't move
        if (Math.abs(error) < angleTolerance) {
            return;
        }

        // Convert degrees into encoder ticks
        int tickAdjustment =
                (int)(error * TICKS_PER_DEGREE);

        // Move relative to current position
        int newTarget =
                turret.getCurrentPosition()
                + tickAdjustment;

        // Clamp to turret limits
        newTarget = Math.max(
                MIN_POSITION,
                Math.min(MAX_POSITION, newTarget)
        );

        turret.setTargetPosition(newTarget);

        // Keep RUN_TO_POSITION active
        turret.setPower(MAX_POWER);
    }

    public int getPosition() {
        return turret.getCurrentPosition();
    }

    public int getTarget() {
        return turret.getTargetPosition();
    }
}
