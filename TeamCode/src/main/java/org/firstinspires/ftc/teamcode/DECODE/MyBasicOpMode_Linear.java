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
@Disabled
@TeleOp(name="does this work", group="Linear OpMode")

public class MyBasicOpMode_Linear extends LinearOpMode {

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
    boolean partyMode = false;

    private boolean hadPressedDPadDown = false;




    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "Webcam 1");
        left = hardwareMap.get(DcMotor.class, "left_drive");
        right = hardwareMap.get(DcMotor.class, "right_drive");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        feederLever = hardwareMap.get(Servo.class, "feederLever");
        pusher = hardwareMap.get(Servo.class, "pusher");
        intake = hardwareMap.get(DcMotor.class, "intakeWheel");
        intake.setDirection(DcMotor.Direction.FORWARD);
        light  = hardwareMap.get(Servo.class, "blink");
        PIDFCoefficients pidf = new PIDFCoefficients(90,0,0,23);
        flywheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER,pidf);
        laserInput = hardwareMap.get(DigitalChannel.class, "distancer");


        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //flywheel.setPower(1.0);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);


        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);


        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();
        runtime.reset();
        pusher.setPosition(1.0);
        boolean hadPressedLeft = false;
        boolean hadPressedRight = false;
        double leftPower = 0;
        double flywheelPower = 0;
        double rightPower = 0;
        double lastPressedTime = -1.0;
        double partyPos = 0.245;
        boolean goingUp = true;
        double dooble = 0.0;
        boolean hadPressedParty = false;
        double lastPressedTimeSpecial = 0;


        while (opModeIsActive()) {

            if (runtime.seconds() < 60+58) {
                LLStatus status = limelight.getStatus();
                telemetry.addData("Name", "%s",
                        status.getName());


                LLResult result = limelight.getLatestResult();
                if (result.isValid()) {
                    if (!partyMode) {
                        light.setPosition(0.5);
                    }
                    // Access general information
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("txnc", result.getTxNC());
                    telemetry.addData("ty", result.getTy());
                    telemetry.addData("tync", result.getTyNC());
                    telemetry.addData("tarea", result.getTa());
                    double distance = Math.pow((result.getTa()/10295.76),-0.5566);
                    telemetry.addData("distance???", distance);

                    if (distance> 110) {
                        if (distance > 190) {
                            flywheelPower = 190 + ((distance-190) * 3);
                        } else {
                            flywheelPower = distance;
                        }
                    } else {
                        flywheelPower = 120;
                    }
                    flywheelPower = 0;

                } else {
                    if (!partyMode) {
                        light.setPosition(0.284);
                    }
                    telemetry.addData("Limelight", "No data available");
                }


                if (gamepad1.dpad_down && !hadPressedDPadDown) {
                    //turns isAiming to "true"
                    isAiming = !isAiming;
                }

                if (aimRobot()) {

                } else {
                    boolean stateHigh = laserInput.getState();
                    if (stateHigh) {
                        telemetry.addLine("You have a ball");
                    } else {
                        telemetry.addLine("You do NOT have a ball");
                    }
                    if (gamepad1.dpadUpWasPressed() && lastPressedTime == -1.0) {
                        feederLever.setPosition(0.0);
                        lastPressedTimeSpecial = runtime.seconds();
                    } else if (lastPressedTime == -1.0 && lastPressedTimeSpecial + 1 < runtime.seconds()) {
                        feederLever.setPosition(1.0);

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

                    if (gamepad1.aWasPressed()) {
                        flywheelPower += 0.05;
                    } else if (gamepad1.bWasPressed()) {
                        flywheelPower -= 0.05;
                    } else if (gamepad1.x) {
                        flywheelPower = 0.55;
                    } else if (gamepad1.y) {
                        flywheelPower = 0.0;
                    }


                    if (gamepad1.left_bumper) {
                        intake.setPower(1.0);
                    } else if (gamepad1.right_bumper) {
                        intake.setPower(-1.0);
                    } else {
                        intake.setPower(0.0);
                    }

                    if (gamepad1.dpadRightWasPressed()) {
                        partyMode = !partyMode;
                    }

                    if (partyMode) {
                        light.setPosition(partyPos);
                    }



                    flywheel.setVelocity(flywheelPower * 2.0 * Math.PI / 60.0, AngleUnit.RADIANS);
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
            }

            telemetry.addData("right",Double.toString(leftPower));
            telemetry.addData("left",Double.toString(rightPower));
            telemetry.addData("stuff", runtime.toString() + Double.toString(lastPressedTime));
            telemetry.addData("power", Double.toString(flywheel.getVelocity(AngleUnit.RADIANS) * 60.0 / (2.0 * Math.PI)));

            telemetry.update();
        }
        limelight.stop();

    }
    private boolean aimRobot() {
        final double Kp = 0.015; // Proportional gain - TUNE THIS!
        final double TARGET_TX = 0.0;
        final double TOLERANCE = 0.5;
        final double MIN_POWER = 0.3; // Add this constant. TUNE to your robot.
        final double MAX_POWER = 0.7; // From your clamping code.

        if (!isAiming) {

            //telemetry.addData("aimRobot isAiming", Boolean.toString(isAiming));
            //telemetry.update();
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
}
