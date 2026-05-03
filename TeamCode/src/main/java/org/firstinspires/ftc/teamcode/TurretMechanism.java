package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.openftc.apriltag.AprilTagDetection;


public class TurretMechanism {
    private DcMotorEx turret;

    private double kP = -0.02690;
    private double kD = 0.12001;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 3.5;
    private final double maxPower = 0.8;
    private double power = 0;

    private final ElapsedTime timer = new ElapsedTime();

    // 🔥 NEW: mode control
    private boolean isResetting = false;

    public void init(HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "turret");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void resetTurret() {
        isResetting = true;

        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.6); // speed of reset
    }

    public void setKP(double newKP) { kP = newKP; }
    public double getKP() { return kP; }

    public void setKD(double newKD) { kD = newKD; }
    public double getKD() { return kD; }

    public void resetTimer() {
        timer.reset();
    }

    public void update(LLResult result) {
        // 🟣 RESET MODE
        if (isResetting) {
            if (!turret.isBusy()) {
                // done resetting → go back to tracking mode
                turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                isResetting = false;
                lastError = 0;
            }
            return; // 🚫 skip Limelight tracking
        }

        // 🔵 NORMAL TRACKING MODE
        double deltaTime = timer.seconds();
        timer.reset();

        if (result == null || !result.isValid()) {
            turret.setPower(0);
            lastError = 0.0;
            return;
        }

        double error = goalX - result.getTx();
        double pTerm = error * kP;

        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = (error - lastError) / deltaTime * kD;
        }

        if (Math.abs(error) < angleTolerance) {
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm, -maxPower, maxPower);
        }

        turret.setPower(power);
        lastError = error;
    }
}