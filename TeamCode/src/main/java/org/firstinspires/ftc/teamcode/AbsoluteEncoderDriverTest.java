package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class AbsoluteEncoderDriverTest extends LinearOpMode {
    AbsoluteEncoderDriver encLeft = null;
    AbsoluteEncoderDriver encRight = null;
    @Override
    public void runOpMode() {
        encLeft = hardwareMap.get(AbsoluteEncoderDriver.class, "as5600Left");
        encRight = hardwareMap.get(AbsoluteEncoderDriver.class, "as5600Right");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Left ticks: ", encLeft.getTicks());
            telemetry.addData("Right ticks: ", encRight.getTicks());
            telemetry.update();
        }

    }
}
