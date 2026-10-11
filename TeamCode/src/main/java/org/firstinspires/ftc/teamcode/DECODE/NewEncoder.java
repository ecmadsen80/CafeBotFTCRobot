package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDevice;

@Disabled
@TeleOp(name = "AS5600 Absolute Encoder Example", group = "Test")
public class NewEncoder extends LinearOpMode {

    // Motor
    private DcMotor motor;

    // AS5600 I2C
    private I2cDevice as5600Device;
    private I2cDeviceSynch as5600;

    // AS5600 Registers
    private static final int AS5600_ADDR = 0x36;
    private static final int ANGLE_REGISTER = 0x0E; // 0x0E + 0x0F = 12-bit angle

    @Override
    public void runOpMode() {

        // ----- Motor Setup -----
        motor = hardwareMap.get(DcMotor.class, "motor");

        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        // ----- AS5600 Setup (Modern SDK approach) -----
        final I2cDevice i2cDevice = hardwareMap.get(I2cDevice.class, "as5600");
        final I2cAddr i2cAddr = I2cAddr.create7bit(AS5600_ADDR);

        // ----- AS5600 Setup (Corrected for SDK 11.0.0+) -----
        as5600 = hardwareMap.get(I2cDeviceSynch.class, "as5600");

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Example motor control
            motor.setPower(gamepad1.left_stick_y);

            // ----- Read AS5600 Angle -----
            byte[] angleBytes = as5600.read(ANGLE_REGISTER, 2);

            int high = angleBytes[0] & 0xFF;
            int low = angleBytes[1] & 0xFF;

            int rawAngle = ((high << 8) | low) & 0x0FFF;   // 12-bit value (0-4095)
            double degrees = rawAngle * 360.0 / 4096.0;

            // ----- Telemetry -----
            telemetry.addData("Raw Angle", rawAngle);
            telemetry.addData("Degrees", degrees);
            telemetry.update();
        }
    }


}
