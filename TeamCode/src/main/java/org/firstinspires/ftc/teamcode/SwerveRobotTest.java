package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(group="BIOBUZZ", name="SwerveRobot test")
public class SwerveRobotTest extends LinearOpMode {

    private SwerveRobot robot = new SwerveRobot();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        waitForStart();

        robot.zeroPods(354.375, 173.32);

        while (opModeIsActive()) {
            robot.driveGamepad(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
        }
    }
}
