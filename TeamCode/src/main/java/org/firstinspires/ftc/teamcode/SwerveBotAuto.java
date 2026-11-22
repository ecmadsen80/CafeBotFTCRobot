package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Autonomous(group = "BIOBUZZ", name = "SwerveBot Auto")
public class SwerveBotAuto extends LinearOpMode {

    SwerveRobot robot = new SwerveRobot();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        waitForStart();
        robot.zeroPods(Constants.LEFT_ZERO_POSITION, Constants.RIGHT_ZERO_POSITION);

        //instead of handling all driving logic in the while loop, we only have the while loop for waiting when the
        //entire auto is finished.

        robot.driveAt(0, 24, 0.5); //drive forward 6 inches
        robot.driveAt(90, 12, 0.5);
        robot.driveAt(180, 24, 0.5);
        robot.driveAt(270, 12, 0.5);

        robot.driveTo(new Pose2D(DistanceUnit.INCH, 12, 0, AngleUnit.DEGREES, 90), 0.5);



    }

}
