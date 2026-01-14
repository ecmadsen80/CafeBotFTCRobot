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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.seattlesolvers.solverslib.controller.PIDFController;
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
@Disabled
@Autonomous(name="Auto (Near Tower) Not working", group="Linear OpMode")

public class UpdatedFinalAutoSad extends LinearOpMode {

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
    PIDFController aimPid = new PIDFController(0.5, 0.0, 0.01, 0.5);
    // ArrayList<Float> myList = new ArrayList<>();


    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftTurn = hardwareMap.get(DcMotorEx.class, "left_turn");
        rightTurn = hardwareMap.get(DcMotorEx.class, "right_turn");
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        intake = hardwareMap.get(DcMotor.class, "intake");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        feederLever = hardwareMap.get(Servo.class, "feederLever");
        limelight = hardwareMap.get(Limelight3A.class, "Webcam 1");
        pusher = hardwareMap.get(DcMotor.class, "rightpusher");
        pusher1 = hardwareMap.get(DcMotor.class, "leftpusher");
        light = hardwareMap.get(Servo.class, "light");

        ElapsedTime timer = new ElapsedTime();


        //intake.setDirection(DcMotor.Direction.FORWARD);light  = hardwareMap.get(Servo.class, "blink");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);


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
        timer.reset();
        // Wait for the game to start (driver presses START)
        waitForStart();
        runtime.reset();
        boolean foundResult = false;
        boolean positionFound = false;
        boolean distanceFound = false;
        boolean isAimed = false;
        double distance = 0;
        double pos = 0.234;
        boolean stop = false;


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            leftTurn.setPower(1.0);
            rightTurn.setPower(1.0);

            if (runtime.seconds() > 60) {
                SHOOTHEBALLS();
                SHOOTHEBALLS();
                stop = true;
            }
            flywheel.setPower(1.0);
            flywheel.setVelocity(2760 / 60 * 28);
            LLResult result = limelight.getLatestResult();
            leftDrive.setPower(0.4);
            rightDrive.setPower(-0.4);

            //Positions Robot at 134cm
            if (!result.isValid() && !foundResult) {
                leftDrive.setPower(0.5);
                rightDrive.setPower(-0.5);
                telemetry.addData("ta:", "no result");
            }
            if (result.isValid()) {
                foundResult = true;
                if (Math.pow((result.getTa() / 10295.76), -0.5566) < 135) {
                    leftDrive.setPower(0.5);
                    rightDrive.setPower(-0.5);

                    telemetry.addData("ta", result.getTa());
                    distance = Math.pow((result.getTa() / 10295.76), -0.5566);
                    //distance *= 2;
                    telemetry.addData("distance:", String.valueOf(distance));
                    telemetry.addData("ta:", result.getTa());
                } else if (Math.pow((result.getTa() / 10295.76), -0.5566) > 135) {
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    telemetry.addData("stop:", "stop");
                    distanceFound = true;
                    isAimed = aimAtTag();
                }
            }

            // The aimAtTag() method will handle turning the robot and will return true when it's done.


            // If aiming is complete, set the positionFound flag to true so the next step can begin.
            if (isAimed) {
                positionFound = true;
            }

            //fires if in correct position
            if (positionFound && !stop) {
                SHOOTHEBALLS();
                SHOOTHEBALLS();
                stop = true;
            }
            telemetry.addData("x", result.getTx());
            telemetry.addData("elapsed time", timer.toString());
            telemetry.update();

            if (stop || (timer.seconds() > 25)) {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                flywheel.setVelocity(0);
                feederLever.setPosition(0);
                leftTurn.setVelocity(0);
                rightTurn.setVelocity(0);
            }
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
        sleepSeconds(1);
        feederLever.setPosition(0.0); //fires one ball
        sleepSeconds(1);
        feederLever.setPosition(1.0);
        intake.setPower(-1.0);
        pusher.setPower(-1.0);
        pusher1.setPower(-1.0);
        sleepSeconds(1);
        feederLever.setPosition(0.0);
        sleepSeconds(1);
        feederLever.setPosition(1.0);

    }

    private double pointAtTag() {
        LLResult result = limelight.getLatestResult();


        if (result != null && result.isValid()) {
            // SolversLib uses calculate(target, current)
            // We want the horizontal offset (tx) to be 0
            double target = result.getTx();


            double pidOutput = aimPid.calculate(0, target);
            return Range.clip(pidOutput, -1.0, 1.0);
        }

        aimPid.reset(); // Clear integral sum when target is lost
        return 0;
    }

    private boolean aimAtTag() {
        final double AIM_TOLERANCE_DEGREES = 5.0; // How close we need to be to count as "aimed"
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {
            // Check if we are already aimed
            if (Math.abs(result.getTx()) < AIM_TOLERANCE_DEGREES) {
                // We are aimed. Stop the motors and report success.
                leftDrive.setPower(0);
                rightDrive.setPower(0);
                return true; // Aiming is complete
            } else if (result.getTx() > AIM_TOLERANCE_DEGREES) {


                leftDrive.setPower(0.3);
                rightDrive.setPower(0.3);
                return false; // Aiming is still in progress
            } else if (result.getTx() < -AIM_TOLERANCE_DEGREES) {
                leftDrive.setPower(-0.3);
                rightDrive.setPower(-0.3);
                return false;
            }

            // If we can't see the tag, we can't aim. Stop motors for safety.
            leftDrive.setPower(0);
            rightDrive.setPower(0);
            return false;
        }
        else {
            return false;
        }

    }
}



