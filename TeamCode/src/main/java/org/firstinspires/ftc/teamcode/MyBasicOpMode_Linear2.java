/*
Copyright (c) 2024 Limelight Vision

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of FIRST nor the names of its contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.robotcontroller.external.samples;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

/*
 * This OpMode illustrates how to use the Limelight3A Vision Sensor.
 *
 * @see <a href="https://limelightvision.io/">Limelight</a>
 *
 * Notes on configuration:
 *
 *   The device presents itself, when plugged into a USB port on a Control Hub as an ethernet
 *   interface.  A DHCP server running on the Limelight automatically assigns the Control Hub an
 *   ip address for the new ethernet interface.
 *
 *   Since the Limelight is plugged into a USB port, it will be listed on the top level configuration
 *   activity along with the Control Hub Portal and other USB devices such as webcams.  Typically
 *   serial numbers are displayed below the device's names.  In the case of the Limelight device, the
 *   Control Hub's assigned ip address for that ethernet interface is used as the "serial number".
 *
 *   Tapping the Limelight's name, transitions to a new screen where the user can rename the Limelight
 *   and specify the Limelight's ip address.  Users should take care not to confuse the ip address of
 *   the Limelight itself, which can be configured through the Limelight settings page via a web browser,
 *   and the ip address the Limelight device assigned the Control Hub and which is displayed in small text
 *   below the name of the Limelight on the top level configuration screen.
 */
@TeleOp(name="does this work", group="Linear OpMode")

public class MyBasicOpMode_Linear2 extends LinearOpMode {

    private Limelight3A limelight;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left = null;
    private DcMotor right = null;
    private DcMotor intake = null;
    private DcMotorEx flywheel = null;
    private Servo feederLever = null;
    private Servo pusher = null;
    public boolean isAiming = false;
    private Servo light;
    private DigitalChannel laserInput;

    private double leftPower = 0; //motor powers
    private double flywheelPower = 0;
    private double rightPower = 0;

    private double partyPos = 0.245; // for party mode
    private boolean goingUp = true;
    private boolean partyMode = false;

    private double lastPressedTimeSpecial = 0;

    private boolean lightOverride = false; // for coloring the light blue if we have a ball.

    @Override
    public void runOpMode() throws InterruptedException
    {
        feederLever = hardwareMap.get(Servo.class, "feederLever");

        intake = hardwareMap.get(DcMotor.class, "intakeWheel");
        intake.setDirection(DcMotor.Direction.FORWARD);

        light  = hardwareMap.get(Servo.class, "blink");

        laserInput = hardwareMap.get(DigitalChannel.class, "distancer");

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        PIDFCoefficients pidf = new PIDFCoefficients(90,0,0,23);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidf);

        telemetry.setMsTransmissionInterval(11);

        limelight = hardwareMap.get(Limelight3A.class, "Webcam 1");
        limelight.pipelineSwitch(0);
        limelight.start();

        left = hardwareMap.get(DcMotor.class, "left_drive");
        right = hardwareMap.get(DcMotor.class, "right_drive");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {

            if (runtime.seconds() < 60+58) {
                pollLimelight();
                if (gamepad1.dpadDownWasPressed()) {
                    //turns isAiming to "true"
                    isAiming = !isAiming;
                }

                if (aimRobot()) {
                } else {
                    pollDistanceSensor();
                    pollFeederLever();
                    pollIntake();

                    dealWithPartyMode();

                    flywheel.setVelocity(flywheelPower * 2.0 * Math.PI / 60.0, AngleUnit.RADIANS);
                    pollMovement();

                } //aiming robot

            } // time limit

            telemetry.addData("Flywheel Velocity", Double.toString(flywheel.getVelocity(AngleUnit.RADIANS) * 60.0 / (2.0 * Math.PI)));

            telemetry.update();
        }
        limelight.stop();

    }
    private boolean aimRobot() {
        final double Kp = 0.011; // Proportional gain - TUNE THIS!
        final double TARGET_TX = 0.0;
        final double TOLERANCE = 0.5;
        final double MIN_POWER = 0.3; // Add this constant. TUNE to your robot.
        final double MAX_POWER = 0.7; // From your clamping code.

        if (!isAiming) {
            return false;
        }
        // 1. Get the latest angle
        LLResult result = limelight.getLatestResult();

        double angleFromTag = result.getTx();
        double error = TARGET_TX - angleFromTag;

        if (Math.abs(error) > TOLERANCE) {
            // 2. Calculate turning power using P-control
            double turnPower = error * Kp;

            // 1. Force Minimum Movement
            if (Math.abs(turnPower) < MIN_POWER) {
                turnPower = Math.signum(turnPower) * MIN_POWER;
            }

            // 2. Limit the power (Clamping)
            turnPower = Range.clip(turnPower, -MAX_POWER, MAX_POWER);
            left.setPower(turnPower);
            right.setPower(-turnPower);
            return true;
        } else {
            // 5. If within tolerance, stop the motors
            left.setPower(0);
            right.setPower(0);
            isAiming = false;
            return false;
        }

    }
    private void pollLimelight() {
        LLStatus status = limelight.getStatus();


        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            if (!partyMode && !lightOverride) {
                light.setPosition(0.5);
            }
            double distance = Math.pow((result.getTa()/10295.76),-0.5566);


            if (distance> 110) {
                if (distance > 190) {
                    flywheelPower = 190 + ((distance-190) * 3);
                } else {
                    flywheelPower = distance;
                }
            } else {
                flywheelPower = 120;
            }
            flywheelPower = 0; //disables flywheel

        } else {
            if (!partyMode && !lightOverride) {
                light.setPosition(0.284);
            }
        }
    }
    private void pollMovement() {
        if ((-gamepad1.right_stick_y <= -0.9 && -gamepad1.left_stick_y <= -0.9)) {
            leftPower = -1;
            rightPower = -1;
        } else if (-gamepad1.right_stick_y >= 0.9 && -gamepad1.left_stick_y >= 0.9) {
            leftPower = 1;
            rightPower = 1;
        } else {
            leftPower  = -gamepad1.right_stick_y * 0.85;
            rightPower = -gamepad1.left_stick_y * 0.85;
        }
        if (gamepad1.right_trigger > 0.4) {
            leftPower *= 0.5;
            rightPower *= 0.5;
        }
        if (!isAiming) {
            left.setPower(leftPower);
            right.setPower(rightPower);
        }
    }
    private void dealWithPartyMode() {
        if (gamepad1.dpadRightWasPressed()) {
            partyMode = !partyMode;
        }

        if (partyMode) {
            light.setPosition(partyPos);
        }
        if (goingUp) {
            partyPos += 0.005;
            if (partyPos > 0.722) {
                goingUp = false;
            }
        } else {
            partyPos -= 0.005;
            if (partyPos < 0.279) {
                goingUp = true;
            }
        }
    }
    private void pollIntake() {
        if (gamepad1.left_bumper) {
            intake.setPower(1.0);
        } else if (gamepad1.right_bumper) {
            intake.setPower(-1.0);
        } else {
            intake.setPower(0.0);
        }
    }
    private void pollFeederLever() {
        if (gamepad1.dpadUpWasPressed()) {
            feederLever.setPosition(0.0);
            lastPressedTimeSpecial = runtime.seconds();
        } else if (lastPressedTimeSpecial + 1 < runtime.seconds()) {
            feederLever.setPosition(1.0);

        }
    }
    private void pollDistanceSensor() {
        boolean stateHigh = laserInput.getState();
        if (stateHigh) {
            telemetry.addLine("You have a ball");
        } else {
            telemetry.addLine("You do NOT have a ball");
        }
        if (gamepad1.x && stateHigh) {
            light.setPosition(0.611);
            lightOverride = true;
        } else {
            lightOverride = false;
        }
    }
}
