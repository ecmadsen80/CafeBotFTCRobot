package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="PARTY BOT!!!", group="Linear OpMode")

public class partybot extends LinearOpMode {

    Servo light = null;
    private DcMotor rightDrive = null;
    private DcMotorEx rightTurn;
    private DcMotor leftDrive = null;
    private DcMotorEx leftTurn;

    @Override
    public void runOpMode() throws InterruptedException {
        light  = hardwareMap.get(Servo.class, "light");
        leftTurn  = hardwareMap.get(DcMotorEx.class, "left_turn");
        rightTurn = hardwareMap.get(DcMotorEx.class, "right_turn");
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        leftTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftTurn.setTargetPosition(0);
        rightTurn.setTargetPosition(0);

        leftTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftTurn.setPower(1.0);
        rightTurn.setPower(1.0);
        waitForStart();
        double pos = 0.234;
        boolean goingUp = true;

        while (opModeIsActive()) {

            leftDrive.setPower(1.0);
            rightDrive.setPower(-1.0);

            if (goingUp) {
                pos += 0.005;
                if (pos > 0.722) {
                    goingUp = false;
                }
            } else {
                pos -= 0.005;
                if (pos < 0.279) {
                    goingUp = true;
                }
            }

            light.setPosition(pos);

        }
    }
}
