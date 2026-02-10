package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

@TeleOp(name = "AS5600 via 2m sensor", group = "Test")
public class AS5600Test extends LinearOpMode {

    private DcMotor motor;
    private I2cDeviceSynch as5600;

    private static final int AS5600_ADDR = 0x36;
    private static final int ANGLE_REGISTER = 0x0E;

    @Override
    public void runOpMode() {

        // ----- Motor Setup -----

        // ----- AS5600 Setup via REV 2m Distance Sensor -----
        Rev2mDistanceSensor dummySensor =
                hardwareMap.get(Rev2mDistanceSensor.class, "as5600");

        as5600 = dummySensor.getDeviceClient();
        as5600.setI2cAddress(I2cAddr.create7bit(AS5600_ADDR));
        as5600.engage();

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double degrees = getAngle();
            double rawAngle = getRawAngle();
            telemetry.addData("Raw Angle", rawAngle);
            telemetry.addData("Degrees", degrees);
            telemetry.update();
        }
    }

    private double getAngle() {
        byte[] angleBytes = as5600.read(ANGLE_REGISTER, 2);

        int high = angleBytes[0] & 0xFF;
        int low = angleBytes[1] & 0xFF;

        int rawAngle = ((high << 8) | low) & 0x0FFF;
        return rawAngle * 360.0 / 4096.0;
    }

    private double getRawAngle(){
        byte[] angleBytes = as5600.read(ANGLE_REGISTER, 2);

        int high = angleBytes[0] & 0xFF;
        int low = angleBytes[1] & 0xFF;

        int rawAngle = ((high << 8) | low) & 0x0FFF;
        return rawAngle;
    }
}
