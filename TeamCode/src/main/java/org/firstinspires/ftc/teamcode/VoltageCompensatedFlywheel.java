package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class VoltageCompensatedFlywheel {
    private final DcMotorEx flywheelLeft;
    private final DcMotorEx flywheelRight;
    private final VoltageSensor voltageSensor;

    // Tuning constants (in Volts and ticks/sec)
    // Adjust kV based on your maximum ticks per second for your UltraPlanetary ratio
    private double kV = 0.0039;
    private double kS = 1.1;
    private double kP = 0.003;
    private double kD = 0.0001;

    private double targetVelocity = 0.0;
    private double lastError = 0.0;
    private final ElapsedTime timer = new ElapsedTime();

    public VoltageCompensatedFlywheel(DcMotorEx flywheelLeft, DcMotorEx flywheelRight, VoltageSensor voltageSensor) {
        this.flywheelLeft = flywheelLeft;
        this.flywheelRight = flywheelRight;
        this.voltageSensor = voltageSensor;

        // Configure motor modes and directions
        flywheelLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheelLeft.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // Set opposing directions so positive power drives them together
        flywheelLeft.setDirection(DcMotorEx.Direction.REVERSE);
        flywheelRight.setDirection(DcMotorEx.Direction.FORWARD);
    }

    public void setTargetVelocity(double velocity) {
        this.targetVelocity = velocity;
    }

    public void adjustKS(double amount) {
        this.kS += amount;
    }

    public void adjustKP(double amount) {
        this.kP += amount;
    }

    public double getKS() {
        return kS;
    }

    public double getKP() {
        return kP;
    }

    public double getkV(){
        return kV;
    }
    // Call this method every loop cycle in your OpMode
    public void update() {
        // If target is 0, cut power immediately
        if (targetVelocity == 0) {
            flywheelLeft.setPower(0);
            flywheelRight.setPower(0);
            lastError = 0;
            return;
        }

        double currentVelocity = flywheelRight.getVelocity(); // Use right as feedback sensor
        double error = targetVelocity - currentVelocity;
        double dt = timer.seconds();
        timer.reset();

        // 1. Feedforward (kS + kV)
        double feedforwardVolts = kS + (kV * targetVelocity);

        // 2. Feedback (PD)
        double derivative = (dt > 0) ? (error - lastError) / dt : 0;
        double feedbackVolts = (kP * error) + (kD * derivative);
        lastError = error;

        // 3. Total commanded voltage & live bus compensation
        double commandedVolts = feedforwardVolts + feedbackVolts;
        double currentBusVoltage = voltageSensor.getVoltage();
        double motorPower = Range.clip(commandedVolts / currentBusVoltage, -1.0, 1.0);

        // 4. Apply power to both motors
        flywheelLeft.setPower(motorPower);
        flywheelRight.setPower(motorPower);
    }
}