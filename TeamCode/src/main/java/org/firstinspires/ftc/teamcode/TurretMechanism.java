package org.firstinspires.ftc.teamcode;

import static java.lang.Math.abs;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class TurretMechanism {

    private DcMotorEx turret;

    // PID constants
    private double kP = 0.013;
    private double kD = 0.0132;

    // Desired tx
    private double goalX = 0;

    // Limits
    private final double maxPower = 0.7;
    private final double minPower = 0.1;

    // State
    private double lastError = 0;
    private double filteredDerivative = 0;
    private double filteredTx = 0;

    private boolean isOverridden = false;
    private boolean isFrozen = false;
    private double movement = 0.0;

    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "turret");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        timer.reset();
    }

    public void freeze() {
        isFrozen = true;
    }

    public void unfreeze() {
        isFrozen = false;
    }

    public boolean getFreezeState() {
        return isFrozen;
    }

    public void override(double power) {

        if (abs(power) < 0.1) {
            movement = 0;
            isOverridden = false;
            isFrozen = false;
            return;
        }

        movement = -power;
        isOverridden = true;
        isFrozen = true;
    }


    public void setKP(double newKP) {
        kP = newKP;
    }

    public void setKD(double newKD) {
        kD = newKD;
    }

    public double getKP() {
        return kP;
    }

    public double getKD() {
        return kD;
    }

    public void resetTimer() {
        timer.reset();
    }

    public void update(LLResult result) {
        if (isFrozen) {
            if (isOverridden) {
                turret.setPower(movement);
                return;
            }
            turret.setPower(0);
            return;
        }
        // Manual override


        // Frozen


        //--------------------------------------
        // Stable delta time
        //--------------------------------------

        double deltaTime =
                Math.max(timer.seconds(),0.01);

        timer.reset();

        //--------------------------------------
        // Lost target
        //--------------------------------------

        if (result == null || !result.isValid()) {

            turret.setPower(0);

            lastError = 0;
            filteredDerivative = 0;

            return;
        }

        //--------------------------------------
        // Smooth camera data
        //--------------------------------------

        double tx = result.getTx();

        // Low-pass filter
        filteredTx =
                filteredTx * 0.7 +
                        tx * 0.3;

        //--------------------------------------
        // PID
        //--------------------------------------

        double error = goalX - filteredTx;

        // P
        double pTerm = error * kP;

        // Raw derivative
        double derivative =
                (error - lastError)
                        / deltaTime;

        // Filter derivative
        filteredDerivative =
                filteredDerivative * 0.8
                        + derivative * 0.2;

        double dTerm =
                filteredDerivative * kD;

        //--------------------------------------
        // Output
        //--------------------------------------

        double power =
                pTerm + dTerm;

// Add safe override assist


        power = Range.clip(
                power,
                -maxPower,
                maxPower
        );

        // Small output deadband
        if (abs(power) < minPower) {
            power = 0;
        }

        turret.setPower(power);

        lastError = error;
    }
}