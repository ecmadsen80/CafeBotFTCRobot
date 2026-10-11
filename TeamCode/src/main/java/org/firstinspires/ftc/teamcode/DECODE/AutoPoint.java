package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name="AutoPoint", group="Linear OpMode")
public class AutoPoint extends LinearOpMode {

    private Limelight3A limelight;
    private Servo lightTurn;

    // Keep last known good angle
    private int RED_APRIL_TAG = 24;
    private int BLUE_APRIL_TAG = 20;
    private double lastError = 0;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(Limelight3A.class, "Webcam 1");
        lightTurn = hardwareMap.get(Servo.class, "lightTurn");

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            double currentServoPos = 0.5; // Start at center
            double TOLERANCE = 3; // Degrees of error to ignore
            double SMOOTHING = 0.4; // Lower = smoother/slower (0.05 to 0.2 is best)

// Inside your while(opModeIsActive) loop:
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double tx = result.getTx(); // Horizontal offset in degrees

                // 1. Tolerance (Deadband) logic
                if (Math.abs(tx) > TOLERANCE) {

                    // 2. Calculate the "Target" position (mapping -30/30 degrees to 0/1 range)
                    double targetPos = 0.5 + (tx / 60.0);

                    // 3. Smoothing (Linear Interpolation)
                    // This moves the servo a fraction of the way to the target each frame
                    currentServoPos = currentServoPos + (targetPos - currentServoPos) * SMOOTHING;

                    lightTurn.setPosition(Range.clip(currentServoPos, 0, 1));
                }
            } else {
                telemetry.addLine("No Target - Holding Position");
            }

            telemetry.addData("Offset", result.getTx());
            telemetry.addData("Servo Pos", currentServoPos);
            telemetry.update();
        }



    }

    private double angleFromAprilTag() { // Function returns the angle the robot is from the AprilTag

        double angleFromTagDeg = 0;
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {

            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults(); // Get the list of all visible tags
            for (LLResultTypes.FiducialResult fr : fiducials) {
                if (fr.getFiducialId() == RED_APRIL_TAG || fr.getFiducialId() == BLUE_APRIL_TAG) {                          //This identifies the Red AprilTag
                    telemetry.addData("Red April Tag", "Found");
                    Pose3D targetPose = fr.getRobotPoseTargetSpace();               //Gets the "pose" of the robot relative to the AprilTag
                    double x = targetPose.getPosition().x;                          //Obtains x, y, and z of the robot relative to the AprilTag
                    double y = targetPose.getPosition().y;
                    double z = targetPose.getPosition().z;
                    double yaw = targetPose.getOrientation().getYaw(AngleUnit.DEGREES);

                    // Bearing from tag to camera in tag plane: x is the "left/right" distance, z is the distance straight out from the Tag
                    angleFromTagDeg = Math.toDegrees(Math.atan2(x, z)) + 180;       //This does the math to calculate the angle of the robot from
                    //from the Tag. Have to add 180 for Red side.

                    //Telemetry data added for troubleshooting and verification
                    /*
                    telemetry.addData("x, y, z, yaw", "%.2f, %.2f, %.2f, %.2f", x, y, z, yaw);
                    telemetry.addData("Target Orientation", targetPose.getOrientation().toString());
                    telemetry.addData("angleFromTagDeg", angleFromTagDeg);

                     */
                    break;

                    // Repeat for the Blue side
                } else if (fr.getFiducialId() == BLUE_APRIL_TAG) {
                    telemetry.addData("Blue April Tag", "Found");

                    Pose3D targetPose = fr.getRobotPoseTargetSpace();

                    double x = targetPose.getPosition().x; // camera X in tag frame
                    double y = targetPose.getPosition().y; // camera Y in tag frame
                    double z = targetPose.getPosition().z;
                    double yaw = targetPose.getOrientation().getYaw(AngleUnit.DEGREES);

                    // Bearing from tag to camera in tag plane (check LL axis docs for sign/axis)
                    angleFromTagDeg = 180 - (Math.toDegrees(Math.atan2(x, z)));

                    /*
                    telemetry.addData("x, y, z, yaw", "%.2f, %.2f, %.2f, %.2f", x, y, z, yaw);
                    telemetry.addData("Target Orientation", targetPose.getOrientation().toString());
                    telemetry.addData("angleFromTagDeg", angleFromTagDeg);

                     */
                    break;
                }
            }
        }

        return angleFromTagDeg;
    }
}
