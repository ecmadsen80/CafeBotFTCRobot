package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="Basic: Linear OpMode", group="Linear OpMode")

public class AprilTest extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "oliver is a duck");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower = 0;
            double rightPower = 0.83333*leftPower;




            /*while (runtime.seconds() < 4){
                leftPower = 0.5;
                rightPower = leftPower;
                leftDrive.setPower(leftPower);
                rightDrive.setPower(rightPower);
            } */

            if (gamepad1.left_trigger > 0.1){
                leftDrive.setPower(0.5*leftPower);
                rightDrive.setPower(0.5*rightPower);
            }
            else if (gamepad1.left_trigger < 0.1){
                leftDrive.setPower(leftPower);
                rightDrive.setPower(rightPower);
            }

            /*leftPower = 0;
            rightPower = 0;
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);*/

            //turnLeft();


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Hi");
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();


        }


    }
    public void turnRight(){
        runtime.reset();
        while (runtime.seconds() < 2){
            double leftPower = 0.5;
            double rightPower = 0;
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
        }
    }
    public void turnLeft(){
        for(int i = 0; i < 3; i++) {
            turnRight();
        }
    }
}
