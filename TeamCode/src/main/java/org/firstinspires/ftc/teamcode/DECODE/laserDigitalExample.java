/*   MIT License
 *   Copyright (c) [2025] [Base 10 Assets, LLC]
 *
 *   Permission is hereby granted, free of charge, to any person obtaining a copy
 *   of this software and associated documentation files (the "Software"), to deal
 *   in the Software without restriction, including without limitation the rights
 *   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 *   copies of the Software, and to permit persons to whom the Software is
 *   furnished to do so, subject to the following conditions:

 *   The above copyright notice and this permission notice shall be included in all
 *   copies or substantial portions of the Software.

 *   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 *   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 *   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 *   SOFTWARE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * goBILDA Laser Distance Sensor Example (Digital Mode)
 *
 * This example shows how to read the digital output of the goBILDA
 * Laser Distance Sensor.
 *
 * In Digital Mode, the sensor outputs either HIGH or LOW depending on
 * whether it detects an object in front of it. The onboard potentiometer
 * adjusts the detection distance from approximately 25mm up to 264mm.
 *
 * This sensor is active-HIGH, meaning the output line goes HIGH (3.3V)
 * when an object is detected, and LOW (0V) when no object is present.
 *
 * Wire the sensor to a Digital port on your Hub and name it "laserDigitalInput"
 * in your Robot Configuration.
 *
 * Display:
 * The current detection state is displayed in telemetry.
 */
//Black – 0
//Red – 0.279
//Orange – 0.333
//Gold – 0.357
//Yellow – 0.388
//Sage – 0.444
//Green – 0.500
//Azure – 0.555
//Blue – 0.611
//Indigo – 0.666
//Violet – 0.722
//White – 1
    @Disabled
@TeleOp(name = "laserDigitalExample")
public class laserDigitalExample extends LinearOpMode {

    private DigitalChannel laserInput;
    private Servo light;
    private DcMotorEx driver;


    @Override
    public void runOpMode() {
        // Get the digital sensor from the hardware map
        //laserInput = hardwareMap.get(DigitalChannel.class, "distancer");
        light  = hardwareMap.get(Servo.class, "light");
        //driver = hardwareMap.get(DcMotorEx.class, "motor1");

        //driver.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //driver.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Set the channel as an input
        //laserInput.setMode(DigitalChannel.Mode.INPUT);
        boolean goingUp = true;
        double pos = 0.0;
        // Wait for the driver to press PLAY
        waitForStart();

        // Loop while the OpMode is active
        while (opModeIsActive()) {
            // Read the sensor state (true = HIGH, false = LOW)
            //boolean stateHigh = laserInput.getState();
            if (goingUp) {
                pos += 0.0005;
                if (pos > 0.722) {
                    goingUp = false;
                }
            } else {
                pos -= 0.0005;
                if (pos < 0.279) {
                    goingUp = true;
                }
            }



            // Active-HIGH: HIGH means an object is detected
            //boolean detected = stateHigh;

            // Display detection state
            if (true) {
                telemetry.addLine("Object detected!");
                light.setPosition(pos);
            } else {
                telemetry.addLine("No object detected");
                light.setPosition(0.0);
            }

            double targetTPS = 0;


            // Apply velocity
            //driver.setVelocity(targetTPS);
            //telemetry.addLine(Double.toString(driver.getVelocity()));
            // Display the raw HIGH/LOW signal for reference
            //telemetry.addData("Raw (HIGH/LOW)", stateHigh);
            telemetry.update();
        }
    }
}
