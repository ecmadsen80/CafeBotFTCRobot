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
@TeleOp(name="OliverTest", group="Linear OpMode")

public class Oliver_LimeLightAlightTest extends LinearOpMode {

    private Limelight3A limelight;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor left = null;
    private DcMotor right = null;
    private DcMotor intake = null;
    private DcMotor flywheel = null;
    private Servo feederLever = null;
    private Servo stopper = null;
    public boolean isAiming = false;
    private boolean hadPressedLeft = false;
    private boolean hadPressedRight = false;
    private boolean hadPressedStopper = false;
    private boolean hadPressedDPadDown = false;

    @Override
    public void runOpMode() throws InterruptedException
    {
        limelight = hardwareMap.get(Limelight3A.class, "Webcam 1");
        left = hardwareMap.get(DcMotor.class, "left_drive");
        right = hardwareMap.get(DcMotor.class, "right_drive");
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        feederLever = hardwareMap.get(Servo.class, "feederLever");
        intake = hardwareMap.get(DcMotor.class, "intakeWheel");
        stopper = hardwareMap.get(Servo.class, "stopper");
        intake.setDirection(DcMotor.Direction.FORWARD);




        //telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(0);

        /*
         * Starts polling for data.  If you neglect to call start(), getLatestResult() will return null.
         */
        limelight.start();
        left.setDirection(DcMotor.Direction.FORWARD);
        right.setDirection(DcMotor.Direction.REVERSE);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        stopper.setPosition(0.75);

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
        waitForStart();
        runtime.reset();

        double leftPower = 0;
        double flywheelPower = 0;
        double rightPower = 0;
        double lastPressedTime = 0.0;
        double stopperLastTime = 0.0;
        double dooble = 0.0;


        while (opModeIsActive()) {


            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());


            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                // Access general information
                telemetry.addData("tx", result.getTx());
                telemetry.addData("txnc", result.getTxNC());
                telemetry.addData("ty", result.getTy());
                telemetry.addData("tync", result.getTyNC());

            } else {
                telemetry.addData("Limelight", "No data available");
            }

            if (gamepad1.dpad_down && !hadPressedDPadDown) {
                //turns isAiming to "true"
                isAiming = !isAiming;
            }

            if (false){
                //aimRobot returns false if isAiming is false and thus should only run if isAiming is true
                //aimRobot returns true while it is in the process of aiming (out of tolerance)
                //aimRobot returns false when it has completed aiming (within tolerance)
                //aimRobot also turns isAiming to false when aiming is complete
            }
            else {

                if (gamepad1.dpad_up) {
                    feederLever.setPosition(0.0);
                    lastPressedTime = runtime.seconds();

                } else if (lastPressedTime+1 < runtime.seconds()) {
                    feederLever.setPosition(1.0);
                    lastPressedTime = 0.0;
                }
                if (gamepad1.a && !hadPressedLeft) {
                    flywheelPower += 0.05;
                } else if (gamepad1.b && !hadPressedRight) {
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

                if (gamepad1.back) {
                    stopper.setPosition(0.2);
                    stopperLastTime = runtime.seconds();
                } else if (stopperLastTime + 2 < runtime.seconds()) {
                    stopper.setPosition(0.75);
                    stopperLastTime = 0.0;
                }


                flywheelPower = Range.clip(flywheelPower, -1.0,1.0);

                flywheel.setPower(flywheelPower);
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
                left.setPower(leftPower);
                right.setPower(rightPower);
            }

            hadPressedDPadDown = gamepad1.dpad_down;
            hadPressedStopper = gamepad1.start;
            hadPressedLeft = gamepad1.a;
            hadPressedRight = gamepad1.b;

            telemetry.addData("right",Double.toString(leftPower));
            telemetry.addData("stopper",Double.toString(stopper.getPosition()));
            telemetry.addData("left",Double.toString(rightPower));
            telemetry.addData("stuff", runtime.toString() + Double.toString(lastPressedTime));
            telemetry.addData("power", Double.toString(flywheelPower));
            telemetry.addData("Aiming State", Boolean.toString(isAiming));
            telemetry.update();


        }
        limelight.stop();
    }


}
