package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(group = "BIOBUZZ", name = "SwerveBot Auto")
public class SwerveBotAuto extends LinearOpMode {

    SwerveRobot robot = new SwerveRobot();
    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        waitForStart();


        //instead of handling all driving logic in the while loop, we only have the while loop for waiting when the
        //entire auto is finished.

    }

}
