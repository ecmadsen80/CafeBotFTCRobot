package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="gottagofast", group="Linear OpMode")

public class gottagofast extends OpMode {
    private DcMotor turret;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotor.class, "turret");
    }
    @Override
    public void loop() {
        if (Math.abs(gamepad1.left_stick_x) > 0.05) {
            turret.setPower(1.0);
        } else {
            turret.setPower(-1.0);
        }
    }
}
