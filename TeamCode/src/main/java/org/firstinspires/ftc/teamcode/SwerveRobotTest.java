package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@TeleOp(group="BIOBUZZ", name="SwerveRobot test")
public class SwerveRobotTest extends LinearOpMode {

    private SwerveRobot robot = new SwerveRobot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        robot.zeroPods(Constants.LEFT_ZERO_POSITION, Constants.RIGHT_ZERO_POSITION);

        while (opModeIsActive()) {

            robot.driveGamepad(gamepad1);
            robot.handleFlywheel(gamepad1);
            Pose2D pos = robot.getPosition();

            double robotX = pos.getX(DistanceUnit.INCH);
            double robotY = pos.getY(DistanceUnit.INCH);
            double robotHeading = pos.getHeading(AngleUnit.DEGREES);
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay(); // Get the field drawing layer

            // 4. DRAW THE ROBOT: A circle for the body, a line pointing to its heading
            fieldOverlay.setStroke("blue");
            fieldOverlay.setStrokeWidth(1);

            // Draw robot perimeter (Assume a typical 18x18 inch robot frame)
            fieldOverlay.strokeRect(robotX-9, robotY-9, 18, 18);
            double headingRad = Math.toRadians(robotHeading);

            double frontX = robotX + Math.cos(headingRad) * 12;
            double frontY = robotY - Math.sin(headingRad) * 12;

            fieldOverlay.strokeLine(robotX, robotY, frontX, frontY);
            dashboard.sendTelemetryPacket(packet);

            double intakePower = 0;
            if (gamepad1.right_trigger > 0) {
                intakePower -= 1.0;
            }
            if (gamepad1.left_trigger > 0) {
                intakePower += 1.0;
            }
            if (gamepad1.x) {
                robot.setGate(1.0);
                intakePower = (intakePower*0.5);
            }
            else {
                robot.setGate(0.0);
            }
            robot.setIntakePower(intakePower);
            telemetry.addLine("Driving");
            telemetry.update();
        }
    }
}
