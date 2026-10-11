/* Copyright (c) 2021 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Disabled
@TeleOp(name="Motor Power Adjuster", group="TeleOp")

public class MotorPowerTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx motorDrive = null;
    private DcMotorEx intakeWheel = null;
    private Servo feederLever = null;

    private double driveRPM = 0.0;
    private final double RPM_STEP = 500;

    private final double SERVO_UP_POS= 0.0;
    private final double SERVO_DOWN_POS = 1.0;

    private static final double TICKS_PER_REVOLUTION = 28;
    private static final double SECONDS_PER_MINUTE = 60.0;



    private boolean wasRightBumperPressed = false;
    private boolean wasLeftBumperPressed = false;

    private boolean wasDpadUpPressed = false;
    private boolean wasDpadDownPressed = false;

    private boolean wasXPressed = false;
    private boolean wasAPressed = false;

    @Override
    public void runOpMode() {

        try {
            motorDrive = hardwareMap.get(DcMotorEx.class, "flyWheel");
        } catch (Exception e) {
            telemetry.addData("Error", "Could not find 'motorDrive'. Check configuration");
            telemetry.update();
        }

        intakeWheel = hardwareMap.get(DcMotorEx.class, "intakeWheel");

        feederLever = hardwareMap.get(Servo.class, "feederLever");

        if (motorDrive != null){
            telemetry.addData("test","found motor");
            motorDrive.setDirection(DcMotor.Direction.REVERSE);
            motorDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motorDrive.setVelocity(0);
        }

        intakeWheel.setDirection(DcMotor.Direction.FORWARD);
        intakeWheel.setPower(0);

        feederLever.setPosition(SERVO_DOWN_POS);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized. Current Power: %.2f", driveRPM);
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            boolean rightBumperPressed = gamepad1.right_bumper;
            boolean leftBumperPressed = gamepad1.left_bumper;
            boolean dpadUpPressed = gamepad1.dpad_up;
            boolean dpadDownPressed = gamepad1.dpad_down;
            boolean xPressed = gamepad1.x;
            boolean aPressed = gamepad1.a;
            double rightTrigger = gamepad1.right_trigger;
            double leftTrigger =  gamepad1.left_trigger;

            if (motorDrive != null){

                if (xPressed && !wasXPressed) {
                    driveRPM = 2500;
                }
                else if (aPressed && !wasAPressed){
                    driveRPM = 0.0;
                }
                if (rightBumperPressed && !wasRightBumperPressed) {
                    driveRPM += RPM_STEP;
                }

                else if (leftBumperPressed && !wasLeftBumperPressed){
                    driveRPM -= RPM_STEP;
                }

                driveRPM = Range.clip(driveRPM, 0.0, 5000);

                //Velocity is = RPMs divided by seconds per minute and multiplied by Ticks per Revolution
                motorDrive.setVelocity((driveRPM/SECONDS_PER_MINUTE)*TICKS_PER_REVOLUTION);


            }



            if (feederLever != null){
                if (dpadUpPressed && !wasDpadUpPressed) {
                    feederLever.setPosition(SERVO_UP_POS);
                }

                else if (dpadDownPressed && !wasDpadDownPressed){
                    feederLever.setPosition(SERVO_DOWN_POS);
                }
            }
            wasRightBumperPressed = rightBumperPressed;
            wasLeftBumperPressed = leftBumperPressed;
            wasDpadUpPressed = dpadUpPressed;
            wasDpadDownPressed = dpadDownPressed;
            wasXPressed = xPressed;
            wasAPressed = aPressed;


            double ticksPerSecond = motorDrive.getVelocity();
            telemetry.addData("tps",Double.toString(ticksPerSecond));


            double motorRPM = (ticksPerSecond / TICKS_PER_REVOLUTION)*SECONDS_PER_MINUTE;


            //Intake motor control; Right trigger is IN
            while (rightTrigger > 0.4){
                intakeWheel.setPower(-rightTrigger);
                rightTrigger = gamepad1.right_trigger;
            }
            while (leftTrigger > 0.4) {
                intakeWheel.setPower(leftTrigger);
                leftTrigger = gamepad1.left_trigger;
            }
            intakeWheel.setPower(0);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motor", "Set RPM: %.2f", driveRPM);
            telemetry.addData("Motor Acutal RPM", "%.1f", motorRPM);
            telemetry.addData("Power", "RB: %.2f | LB: -%.2f", RPM_STEP, RPM_STEP);

            telemetry.update();
        }
    }
}