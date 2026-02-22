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
    private I2cDeviceSynch as5600Right;
    private I2cDeviceSynch as5600Left;

    private static final int AS5600_ADDR = 0x36;
    private static final int ANGLE_REGISTER = 0x0E;
    int LEFT_ZERO = 126;
    int RIGHT_ZERO = 26;

    @Override
    public void runOpMode() {

        // ----- Motor Setup -----

        // ----- AS5600 Setup via REV 2m Distance Sensor -----
        Rev2mDistanceSensor dummySensor =
                hardwareMap.get(Rev2mDistanceSensor.class, "as5600Right");
        Rev2mDistanceSensor dummySensor2 =
                hardwareMap.get(Rev2mDistanceSensor.class, "as5600Left");


        as5600Left = dummySensor.getDeviceClient();
        as5600Left.setI2cAddress(I2cAddr.create7bit(AS5600_ADDR));
        as5600Left.engage();

        as5600Right = dummySensor2.getDeviceClient();
        as5600Right.setI2cAddress(I2cAddr.create7bit(AS5600_ADDR));
        as5600Right.engage();

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();


        while (opModeIsActive()) {

            double degrees = getAngle(as5600Left);
            double rawAngle = getRawAngle(as5600Left);
            telemetry.addData("Raw Angle Left", rawAngle);
            telemetry.addData("Degrees Left", degrees);
            degrees = getAngle(as5600Right);
            rawAngle = getRawAngle(as5600Right);
            telemetry.addData("Raw Angle Right", rawAngle);
            telemetry.addData("Degrees Right", degrees);
            telemetry.update();
        }
    }

    private double getAngle(I2cDeviceSynch as5600) {
        byte[] angleBytes = as5600.read(ANGLE_REGISTER, 2);

        int high = angleBytes[0] & 0xFF;
        int low = angleBytes[1] & 0xFF;

        int rawAngle = ((high << 8) | low) & 0x0FFF;
        return rawAngle * 360.0 / 4096.0;
    }

    private double getRawAngle(I2cDeviceSynch as5600){
        byte[] angleBytes = as5600.read(ANGLE_REGISTER, 2);

        int high = angleBytes[0] & 0xFF;
        int low = angleBytes[1] & 0xFF;

        int rawAngle = ((high << 8) | low) & 0x0FFF;
        return rawAngle;
    }
}
