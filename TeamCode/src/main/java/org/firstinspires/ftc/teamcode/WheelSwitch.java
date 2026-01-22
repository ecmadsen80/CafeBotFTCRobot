package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Wheel switcher", group="Linear OpMode")
public class WheelSwitch extends LinearOpMode {

    private DcMotorEx leftDrive;
    private DcMotorEx leftTurn;
    private DcMotorEx rightDrive;
    private DcMotorEx rightTurn;

    // CHANGE THIS if your motor has a different encoder count
    private static final double TICKS_PER_REV = 751.8;
    private static final double TURN_180_TICKS = TICKS_PER_REV / 2;

    @Override
    public void runOpMode() {

        leftDrive  = hardwareMap.get(DcMotorEx.class, "left_drive");
        leftTurn   = hardwareMap.get(DcMotorEx.class, "left_turn");
        rightDrive = hardwareMap.get(DcMotorEx.class, "right_drive");
        rightTurn  = hardwareMap.get(DcMotorEx.class, "right_turn");

        leftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftTurn.setTargetPosition(0);
        rightTurn.setTargetPosition(0);

        leftTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        leftTurn.setPower(0.5);
        rightTurn.setPower(0.5);

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.xWasPressed()) {
                // Flip RIGHT wheel 180°
                double newTarget = rightTurn.getTargetPosition() + TURN_180_TICKS;
                rightTurn.setTargetPosition((int)newTarget);
            }

            if (gamepad1.bWasPressed()) {
                // Flip LEFT wheel 180°
                double newTarget = leftTurn.getTargetPosition() + TURN_180_TICKS;
                leftTurn.setTargetPosition((int)newTarget);
            }

            telemetry.addData("Left Turn Pos", leftTurn.getCurrentPosition());
            telemetry.addData("Right Turn Pos", rightTurn.getCurrentPosition());
            telemetry.update();
        }
    }
}
