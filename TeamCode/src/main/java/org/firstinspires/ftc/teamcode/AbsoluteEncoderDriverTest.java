package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group = "BIOBUZZ", name = "Encoder driver testing")
public class AbsoluteEncoderDriverTest extends LinearOpMode {
    AbsoluteEncoderDriver encLeft = null;
    AbsoluteEncoderDriver encRight = null;
    @Override
    public void runOpMode() {
        encLeft = hardwareMap.get(AbsoluteEncoderDriver.class, "as5600Left");
        encRight = hardwareMap.get(AbsoluteEncoderDriver.class, "as5600Right");

        waitForStart();


        while (opModeIsActive()) {
            encLeft.update();
            encRight.update();
            telemetry.addData("Left ticks: ", encLeft.getTicks());
            telemetry.addData("Right ticks: ", encRight.getTicks());
            telemetry.update();
        }

    }
}
