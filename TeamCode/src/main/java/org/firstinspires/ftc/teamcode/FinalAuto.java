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

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.ArrayList;
import java.util.List;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
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

@Autonomous(name="State Auto", group="Linear OpMode")

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
    private DcMotor primaryIntake = null;
    private DcMotor secondaryIntake = null;

    private DcMotor pusher = null;
    private DcMotor pusher1 = null;
    private DcMotor inPusher;
    private DcMotor upPusher;
    public boolean isAiming = false;

    static final double TURN_TICKS_PER_REV = 751.8; //gobuilda 5204-8002-0027
    private Servo light;
    private  PIDFCoefficients pidf = new PIDFCoefficients(110,0,0,14);
    // ArrayList<Float> myList = new ArrayList<>();

    private I2cDeviceSynch as5600Left;
    private I2cDeviceSynch as5600Right;

    private static final int AS5600_ADDR = 0x36;
    private static final int ANGLE_REGISTER = 0x0E;

    private static final double LEFT_ZERO_POSITION = 172.3;
    private static final double RIGHT_ZERO_POSITION = 151.6;

    private TurretMechanism turret = new TurretMechanism();


    @Override
    public void runOpMode() {


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        turret.init(hardwareMap);
        turret.setKP(0.005);
        turret.setKD(0.005);

        leftTurn  = hardwareMap.get(DcMotorEx.class, "left_turn");
        rightTurn = hardwareMap.get(DcMotorEx.class, "right_turn");
        leftDrive  = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        primaryIntake = hardwareMap.get(DcMotor.class, "primaryIntake");
        secondaryIntake = hardwareMap.get(DcMotor.class, "secondaryIntake");
        primaryIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");

        limelight = hardwareMap.get(Limelight3A.class, "Webcam 1");
        light = hardwareMap.get(Servo.class, "light");


        //intake.setDirection(DcMotor.Direction.FORWARD);light  = hardwareMap.get(Servo.class, "blink");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.FORWARD);
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

        Rev2mDistanceSensor dummySensorL =
                hardwareMap.get(Rev2mDistanceSensor.class, "as5600Left");
        Rev2mDistanceSensor dummySensorR =
                hardwareMap.get(Rev2mDistanceSensor.class, "as5600Right");

        as5600Left = dummySensorL.getDeviceClient();
        as5600Left.setI2cAddress(I2cAddr.create7bit(AS5600_ADDR));
        as5600Left.engage();

        as5600Right = dummySensorR.getDeviceClient();
        as5600Right.setI2cAddress(I2cAddr.create7bit(AS5600_ADDR));
        as5600Right.engage();




        limelight.pipelineSwitch(0);
        limelight.start();
        int pointer = 0;
        boolean pointerIncreased = true;
        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);
        //feederLever.setPosition(1.0);

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


        double currentAngle = getAngle(as5600Left);
        double zeroAngle = LEFT_ZERO_POSITION;

// Shortest signed angle error in degrees [-180, 180)
        double angleError =
                ((zeroAngle - currentAngle + 540) % 360) - 180;

// Convert degrees → motor ticks
        int deltaTicks = (int) Math.round(
                angleError / 360.0 * TURN_TICKS_PER_REV
        );

// Move RELATIVE to current motor position
        leftTurn.setTargetPosition(
                leftTurn.getCurrentPosition() - deltaTicks
        );


        leftTurn.setVelocity(2500);
        currentAngle = getAngle(as5600Right);
        zeroAngle = RIGHT_ZERO_POSITION;

        angleError =
                ((zeroAngle - currentAngle + 540) % 360) - 180;

        deltaTicks = (int) Math.round(
                angleError / 360.0 * TURN_TICKS_PER_REV
        );

        rightTurn.setTargetPosition(
                rightTurn.getCurrentPosition() - deltaTicks
        );


        rightTurn.setVelocity(2500);

        while ((leftTurn.isBusy() || rightTurn.isBusy() ) && runtime.seconds() < 6) {
            telemetry.addLine("Aligning wheels...");
            telemetry.update();
        }

        leftTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightTurn.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftTurn.setTargetPosition(0);
        rightTurn.setTargetPosition(0);
        leftTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightTurn.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (runtime.seconds() > 8 && !stop) {
                leftDrive.setPower(0.0);
                rightDrive.setPower(0.0);
                primaryIntake.setPower(-1.0);
                secondaryIntake.setPower(-1.0);



            }
            flywheel.setPower(1.0);
            distance = distanceFromTag();
            double targetRPM  = 625.181*distance + 1920.281;
            flywheel.setVelocity((targetRPM/60) * 28.0);
            LLResult result = limelight.getLatestResult();
            //leftDrive.setPower(0.3);
            //rightDrive.setPower(-0.3);
            turret.update(result);
            //Positions Robot at 134cm
            if (!result.isValid() && !foundResult && !stop){
                leftDrive.setPower(0.35);
                rightDrive.setPower(-0.35);
                telemetry.addData("ta:", "no result");
            }
            if(result.isValid() && !stop){
                foundResult = true;
                if (distance < 0.35){
                    leftDrive.setPower(0.35);
                    rightDrive.setPower(-0.35);

                    telemetry.addData("ta", result.getTa());
                    distance = distanceFromTag();
                    telemetry.addData("distance:", String.valueOf(distance));
                    telemetry.addData("ta:", result.getTa());
                }
                else if (distance > 0.35){
                    leftDrive.setPower(0);
                    rightDrive.setPower(0);
                    telemetry.addData("stop:", "stop");
                    positionFound = true;
                }
            }

            //fires if in correct position
            if (positionFound && !stop) {
                leftDrive.setPower(0.0);
                rightDrive.setPower(0.0);
                primaryIntake.setPower(-1.0);
                secondaryIntake.setPower(-1.0);

            }

            telemetry.update();
        }
        limelight.stop();
    }

    private double distanceFromTag() {
        LLResult result = limelight.getLatestResult();
        double distance = 0;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults(); // Get the list of all visible tags
        for (LLResultTypes.FiducialResult fr : fiducials) {
                Pose3D targetPose = fr.getRobotPoseTargetSpace();               //Gets the "pose" of the robot relative to the AprilTag
                double x = targetPose.getPosition().x;                          //Obtains x, y, and z of the robot relative to the AprilTag
                double y = targetPose.getPosition().y;
                double z = targetPose.getPosition().z;

                telemetry.addData("April Tag", "Found");
                distance = Math.sqrt(Math.pow(x, 2) + Math.pow(z, 2));
                telemetry.addData("distanceFromTag", distance);
                telemetry.addData("Formula Distance", Math.pow((result.getTa() / 9946.27), -0.560091));
        }
        return distance;
    }

    private double getAngle(I2cDeviceSynch as5600) {
        byte[] angleBytes = as5600.read(ANGLE_REGISTER, 2);

        int high = angleBytes[0] & 0xFF;
        int low = angleBytes[1] & 0xFF;

        int rawAngle = ((high << 8) | low) & 0x0FFF;
        return rawAngle * 360.0 / 4096.0;
    }


}