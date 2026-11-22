package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(group="BIOBUZZ", name="SwerveRobot auto")
public class SwerveRobotAuto extends LinearOpMode {
  private SwerveRobot robot = new SwerveRobot();

  public void runOpMode() {
    robot.init(hardwareMap, telemetry);
    robot.setBrakingMode(DcMotor.ZeroPowerBehavior.BRAKE);

    waitForStart();

    robot.zeroPods(Constants.LEFT_ZERO_POSITION, Constants.RIGHT_ZERO_POSITION);
    while (opModeIsActive()) {
      //drive drive drive!
    }
  }
}
