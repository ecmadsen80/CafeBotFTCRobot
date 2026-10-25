package org.firstinspires.ftc.teamcode;


import static java.lang.Math.abs;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.Locale;

@TeleOp(group="BIOBUZZ", name="BIOBUZZ-Pushbot")
public class Pushbot extends LinearOpMode {

    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor flywheel = null;
    ElapsedTime time = new ElapsedTime();
    private GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");

        FtcDashboard dashboard = FtcDashboard.getInstance();

        odo.setOffsets(170, 170, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        time.reset();
        while (time.time() < 0.5) {}
        time.reset();
        telemetry.addLine("Initialized");
        odo.update();


        odo.setPosition(new Pose2D(DistanceUnit.INCH, -54, 48, AngleUnit.DEGREES, 180));
        odo.update();
        waitForStart();
        while (opModeIsActive()) {
            odo.update();
            leftMotor.setPower(gamepad1.right_stick_y);
            rightMotor.setPower(gamepad1.left_stick_y);
            flywheel.setPower(flywheel.getPower() + ((gamepad1.dpadUpWasPressed() ? 1 : 0) * 0.05) - ((gamepad1.dpadDownWasPressed() ? 1 : 0) * 0.05));
            Pose2D pos = odo.getPosition();
            double robotX = pos.getX(DistanceUnit.INCH);
            double robotY = pos.getY(DistanceUnit.INCH);
            double robotHeading = pos.getHeading(AngleUnit.DEGREES);
            TelemetryPacket packet = new TelemetryPacket();
            Canvas fieldOverlay = packet.fieldOverlay(); // Get the field drawing layer

            // 4. DRAW THE ROBOT: A circle for the body, a line pointing to its heading
            fieldOverlay.setStroke("blue");
            fieldOverlay.setStrokeWidth(1);

            // Draw robot perimeter (Assume a typical 18x18 inch robot frame)
            fieldOverlay.strokeCircle(robotX, robotY, 9);

            // Draw a line indicating which direction the front of the robot is facing
            double vectorLength = 12.0; // 12-inch pointer line
            double frontX = robotX + Math.cos(Math.toRadians(robotHeading)) * vectorLength;
            double frontY = robotY + Math.sin(Math.toRadians(robotHeading)) * vectorLength;
            fieldOverlay.strokeLine(robotX, robotY, frontX, frontY);

            // 5. Send numerical coordinates text alongside the visual canvas map
            packet.put("Field Position X (in)", robotX);
            packet.put("Field Position Y (in)", robotY);

            // 6. Push the drawing instructions over Wi-Fi to your browser page
            dashboard.sendTelemetryPacket(packet);
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, Heading: %.3f}", pos.getX(DistanceUnit.INCH), pos.getY(DistanceUnit.INCH), pos.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Position", data);

            telemetry.addData("Flywheel: ", flywheel.getPower());
            telemetry.update();
 ;
        }

    }
}
