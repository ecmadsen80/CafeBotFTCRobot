/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
/*flywheel.setPower(0.75);
            if (pointer < myList.size()) {

            if (myList.get(pointer+2) == 254.333) {
                feederLever.setPosition(0.0);
                delay(3);
            } else if (runtime.milliseconds() < myList.get(pointer+2)) {
                //set power.
                rightDrive.setPower(myList.get(pointer+1));
                leftDrive.setPower(myList.get(pointer));
            } else {
                    runtime.reset();
                    pointerIncreased = true;
                    pointer += 3 ;
                    feederLever.setPosition(1.0);
                    if (pointer < myList.size()) {
                    rightDrive.setPower(myList.get(pointer+1));
                    leftDrive.setPower(myList.get(pointer));
                    }

                    }

            } else {
                rightDrive.setPower(0.0);
                leftDrive.setPower(0.0);
            }

*/

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

@Autonomous(name="Auto Tower Updated Monday", group="Linear OpMode")

public class FinalAuto extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive = null;
    private Limelight3A limelight;
    private DcMotor rightDrive = null;
    private DcMotorEx rightTurn;
    private DcMotorEx leftTurn;
    private DcMotorEx flywheel = null;
    private DcMotor intake = null;
    private Servo feederLever = null;
    private DcMotor pusher = null;
    private DcMotor pusher1 = null;
    public boolean isAiming = false;
    private Servo light;
    private static int TARGET_RPM = 2400;
    private static double PUSHER_POWER = 0.7;
    private PIDFCoefficients pidf = new PIDFCoefficients(110,0,0,14);
    private DigitalChannel laserInput;

    // P(roportional) gain for our simple aiming controller.
    private static final double AIM_KP = 0.01;
    // ...
    // ArrayList<Float> myList = new ArrayList<>();


    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftTurn  = hardwareMap.get(DcMotorEx.class, "left_turn");
        rightTurn = hardwareMap.get(DcMotorEx.class, "right_turn");
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        feederLever = hardwareMap.get(Servo.class, "feederLever");
        limelight = hardwareMap.get(Limelight3A.class, "Webcam 1");
        pusher = hardwareMap.get(DcMotor.class, "rightpusher");
        pusher1 = hardwareMap.get(DcMotor.class, "leftpusher");
        light = hardwareMap.get(Servo.class, "light");
        laserInput = hardwareMap.get(DigitalChannel.class, "distancer");


        //intake.setDirection(DcMotor.Direction.FORWARD);light  = hardwareMap.get(Servo.class, "blink");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidf);


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

        feederLever.setPosition(0.0);

        laserInput.setMode(DigitalChannel.Mode.INPUT);

        limelight.start();
        limelight.pipelineSwitch(0);
        int pointer = 0;
        boolean pointerIncreased = true;
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        //feederLever.setPosition(1.0);
        feederLever.setPosition(0.0);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        boolean foundResult = false;
        boolean positionFound = false;
        double distance = 0;
        double pos = 0.234;
        boolean stop = false;



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /*
            if (runtime.seconds() > 25 && !stop) {
                leftDrive.setPower(0.0);
                rightDrive.setPower(0.0);
                SHOOTHEBALLS();
                SHOOTHEBALLS();
                intake.setPower(0.0);
                pusher.setPower(0.0);
                pusher1.setPower(0.0);
                feederLever.setPosition(1.0);
                stop = true;
            }
            */

            //flywheel.setPower(1.0);
            flywheel.setVelocity(TARGET_RPM/60*28);
            LLResult result = limelight.getLatestResult();
            //leftDrive.setPower(0.3);
            //rightDrive.setPower(-0.3);

            //Positions Robot at 134cm
            if (!result.isValid() && !foundResult && !stop){
                leftDrive.setPower(0.5);
                rightDrive.setPower(-0.5);
                telemetry.addData("ta:", "no result");
            }
            if(result.isValid()){
                foundResult = true;
                if (Math.pow((result.getTa()/10295.76),-0.5566) < 100){
                    leftDrive.setPower(0.5);
                    rightDrive.setPower(-0.5);

                    telemetry.addData("ta", result.getTa());
                    distance = Math.pow((result.getTa()/10295.76),-0.5566);
                    distance *= 2;
                    telemetry.addData("distance:", String.valueOf(distance));
                    telemetry.addData("ta:", result.getTa());
                }
                else if (Math.pow((result.getTa()/10295.76),-0.5566) > 100){
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    telemetry.addData("position found:", "true");
                    positionFound = true;
                }
            }

            //fires if in correct position
            if (positionFound && !stop){
                while (!aimAtTag()) {
                    sleep(20);
                }
                SHOOTHEBALLS();
                SHOOTHEBALLS();
                SHOOTHEBALLS();
                intake.setPower(0.0);
                pusher.setPower(0.0);
                pusher1.setPower(0.0);
                feederLever.setPosition(0);
                stop = true;
            }
            telemetry.addData("tx", result.getTx());
            telemetry.update();
        }

        limelight.stop();
    }

    public static void sleepSeconds(int seconds) {
        try {
            Thread.sleep(seconds * 1000L);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    private void SHOOTHEBALLS() {
        // This method will now reliably fire ONE ball and then load the NEXT one.

        // --- Step 1: Wait for the flywheel to be at the correct speed ---
        waitForFlywheelStable();

        // --- Step 2: Fire the ball that is currently loaded ---
        feederLever.setPosition(1.0); // Fires the ball
        sleep(1000);                   // Wait for the servo to move
        feederLever.setPosition(0);   // Reset the feeder
        sleep(1000);                   // Wait for the servo to retract fully

        // --- Step 3: Run the intake and wait for the NEXT ball to be loaded ---
        loadNextBall();
    }

    /**
     * Helper method that turns on the intake and waits for a new ball to be detected.
     */
    private void loadNextBall() {
        intake.setPower(-1.0);
        pusher.setPower(-PUSHER_POWER);
        pusher1.setPower(-PUSHER_POWER);

        ElapsedTime intakeTimer = new ElapsedTime();
        // This loop will run until a new ball is detected, with a 5-second timeout for safety.
        while (opModeIsActive() && intakeTimer.seconds() < 5) {
            // IMPORTANT: Re-read the sensor INSIDE the loop
            if (laserInput.getState()) {
                // Ball has been detected, break out of the intake loop.
                break;
            }
            sleep(100); // Small delay to not spam the CPU
        }

        // Turn off the intake motors after loading or timeout
        intake.setPower(0);
        pusher.setPower(0);
        pusher1.setPower(0);
    }

    /**
     * Helper method to wait until the flywheel RPM is stable.
     */
    private void waitForFlywheelStable() {
        // This loop pauses execution until the flywheel is within 5% of the target RPM.
        while (opModeIsActive() && TARGET_RPM > 0) {
            double currentRPM = flywheel.getVelocity() / 28 * 60;
            if (Math.abs((currentRPM - TARGET_RPM) / TARGET_RPM) <= 0.05) {
                // Speed is stable, we can exit the loop.
                break;
            }
            sleep(50); // Wait 50ms before re-checking.
        }
    }

    private boolean aimAtTag() {
        final double AIM_TOLERANCE_DEGREES = 1.0; // Your desired tolerance
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // The "error" is the horizontal angle to the tag. We want this to be 0.
            double error = result.getTx();

            // Check if we are already aimed
            if (Math.abs(error) < AIM_TOLERANCE_DEGREES) {
                // We are aimed. Stop the motors and report success.
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                return true; // Aiming is complete
            } else {
                // Calculate the proportional turn power.
                // The power will be high when the error is large, and low when it's small.
                double turnPower = Range.clip(error * AIM_KP, -0.4, 0.4); // Clamp max speed to 0.4

                // Apply power to the motors to turn the robot.
                // Remember your robot's unique turning: both motors forward to turn RIGHT.
                if (error > 0) { // Tag is to the RIGHT, so turn RIGHT
                    leftDrive.setPower(-turnPower);
                    rightDrive.setPower(-turnPower);
                } else { // Tag is to the LEFT, so turn LEFT
                    leftDrive.setPower(-turnPower);
                    rightDrive.setPower(-turnPower);
                }
                return false; // Aiming is still in progress
            }
        }

        // If we can't see the tag, stop for safety.
        telemetry.addLine("No tag detected upon aiming");
        leftDrive.setPower(0);
        rightDrive.setPower(0);
        return false;
    }


}