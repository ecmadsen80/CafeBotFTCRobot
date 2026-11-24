package org.firstinspires.ftc.teamcode;


import static java.lang.Math.PI;
import static java.lang.Math.abs;

import androidx.annotation.NonNull;

import java.lang.Math;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

public class SwerveRobot {
    private SwervePod podLeft = new SwervePod();
    private SwervePod podRight = new SwervePod();
    private Intake intake = new Intake();
    private DashboardVisualizer visualizer;

    private DcMotorEx flywheelRight = null;
    private DcMotorEx flywheelLeft = null;
    private VoltageSensor batteryVoltageSensor;
    private VoltageCompensatedFlywheel voltageCompensatedFlywheel;
    private Servo gate;
    private GoBildaPinpointDriver odo;
    private ElapsedTime time = new ElapsedTime();
    private Telemetry telemetry;
    private int targetRPM = 0;
    private PIDFCoefficients pidfFullWeight = new PIDFCoefficients(100,0.0, 0.0, 10);


    public void init(HardwareMap hw, Telemetry t) {
        flywheelRight = hw.get(DcMotorEx.class, "flywheelRight");
        flywheelLeft = hw.get(DcMotorEx.class, "flywheelLeft");
        podLeft.init(hw, "left_turn", "left_drive", "as5600Left");
        podRight.init(hw, "right_turn", "right_drive", "as5600Right");
        intake.init(hw);
        gate = hw.get(Servo.class, "gate");
        gate.setPosition(0.0);
        telemetry = t;
        odo = hw.get(GoBildaPinpointDriver.class, "odo");
        visualizer = new DashboardVisualizer(odo);
        batteryVoltageSensor = hw.voltageSensor.iterator().next();
        

        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfFullWeight);
        voltageCompensatedFlywheel = new VoltageCompensatedFlywheel(flywheelLeft, flywheelRight, batteryVoltageSensor);
        odo.setOffsets(-20, 8, DistanceUnit.CM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();

        time.reset();
        while (time.time() < 0.5) {} //give IMU time to reset.
        time.reset();
        telemetry.addLine("Initialized");
        odo.update();


        odo.setPosition(new Pose2D(DistanceUnit.INCH, 21, 63, AngleUnit.DEGREES, 90)); // On blue side,
        odo.update();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
    }

    public void zeroPods(double leftZero, double rightZero) {
        telemetry.addLine("Aligning motors...");
        telemetry.update();
        podLeft.zero(leftZero);
        podRight.zero(rightZero);

    }

    public void setBrakingMode(DcMotor.ZeroPowerBehavior b) {
        podLeft.setBrakingMode(b);
        podRight.setBrakingMode(b);
    }
    //======================================================================//
    // ALL CODE IN driveAt IS THEORETICAL AND SHOULD BE TESTED BEFORE A MEET//
    //======================================================================//
    //***ONLY TO BE USED IN AUTO***//


    public void driveAt(double targetAngle, double amount, double speed) {
        // 1. Record starting position using odometry
        odo.update();
        visualizer.update();
        double robotStartX = odo.getPosX(DistanceUnit.INCH);
        double robotStartY = odo.getPosY(DistanceUnit.INCH);

        double distanceTraveled = 0;

        // Get current encoder positions for pods
        int leftCurrentTicks = podLeft.getMotorAngle();
        int rightCurrentTicks = podRight.getMotorAngle();

        // Convert to degrees
        int leftCurrentDegrees = (int)((leftCurrentTicks / (double)Constants.GOBILDA_ENCODER_TICKS) * 360);
        int rightCurrentDegrees = (int)((rightCurrentTicks / (double)Constants.GOBILDA_ENCODER_TICKS) * 360);

        double targetAngleLeft = targetAngle;
        double targetAngleRight = targetAngle;

        // Optimize LEFT pod
        double driveMultLeft = 1.0;
        double diffLeft = closestAngle(targetAngleLeft, leftCurrentDegrees);
        if (Math.abs(diffLeft) > 90) {
            driveMultLeft = -1.0;
            targetAngleLeft = (targetAngleLeft + 180) % 360;
        }

        // Optimize RIGHT pod
        double driveMultRight = 1.0;
        double diffRight = closestAngle(targetAngleRight, rightCurrentDegrees);
        if (Math.abs(diffRight) > 90) {
            driveMultRight = -1.0;
            targetAngleRight = (targetAngleRight + 180) % 360;
        }

        // Calculate final motor targets
        double moveLeft = closestAngle(targetAngleLeft, leftCurrentDegrees) + leftCurrentDegrees;
        double moveRight = closestAngle(targetAngleRight, rightCurrentDegrees) + rightCurrentDegrees;

        // Set pod angles and scaled drive speeds continuously
        podLeft.setAngle((int)((moveLeft / 360.0) * Constants.GOBILDA_ENCODER_TICKS));
        podRight.setAngle((int)((moveRight / 360.0) * Constants.GOBILDA_ENCODER_TICKS));
        while (podRight.isBusy() || podLeft.isBusy()) {}

        // 2. Continuous control loop
        while (distanceTraveled < amount) {
            odo.update();
            visualizer.update();
            double currentX = odo.getPosX(DistanceUnit.INCH);
            double currentY = odo.getPosY(DistanceUnit.INCH);
            distanceTraveled = Math.hypot(currentX - robotStartX, currentY - robotStartY);

            // Calculate remaining distance for deceleration scaling
            double remainingDistance = amount - distanceTraveled;

            // Scale speed down as we get close (e.g., within 4 inches) to prevent overshoot
            // Minimum speed (0.15) ensures it overcomes static friction to reach the target
            double currentSpeed = speed;
            if (remainingDistance < 4.0) {
                currentSpeed = Math.max(speed * (remainingDistance / 4.0), 0.15);
            }

            // If we've reached the target distance, break out of the loop
            if (distanceTraveled >= amount) {
                break;
            }




            podLeft.setDrive(currentSpeed * driveMultLeft);
            podRight.setDrive(currentSpeed * driveMultRight);
        }

        // 3. Stop drive motors cleanly
        podLeft.setDrive(0);
        podRight.setDrive(0);
        odo.update();
        visualizer.update();
        while (Math.hypot(odo.getVelX(DistanceUnit.INCH), odo.getVelY(DistanceUnit.INCH)) > 0.3) {odo.update(); visualizer.update();}
    }

    //driveTo code is also theoretical and should be tested before a meet.
    //There are no failsafes in either driveTo or driveAt, meaning that if the robot hits a wall or is stalled in
    //any other way there is no way to recover the robot and continue program operation.

    public void driveTo(Pose2D to, double speed) {
        double toX = to.getX(DistanceUnit.INCH);
        double toY = to.getY(DistanceUnit.INCH);
        double dx = toX-odo.getPosX(DistanceUnit.INCH);
        double dy = toY-odo.getPosY(DistanceUnit.INCH);
        double distance = Math.hypot(dx, dy);
        double angle = Math.toDegrees(Math.atan2(dx, dy));
        driveAt(angle, distance, speed);
    }

    public void logOdometry() {

        telemetry.addData("Odo posX", odo.getPosX(DistanceUnit.INCH));
        telemetry.addData("Odo posY", odo.getPosY(DistanceUnit.INCH));
        telemetry.addData("Odo heading", odo.getHeading(AngleUnit.DEGREES));
    }
    public Pose2D getPosition() {
        return odo.getPosition();
    }


    public void driveGamepad(Gamepad gamepad) {
        odo.update();
        visualizer.update();

        double lx = gamepad.left_stick_x;
        double ly = -gamepad.left_stick_y;
        double turn = gamepad.right_stick_x;
        double leftRotationX = 0;   // Vertical component of rotation for left pod
        double leftRotationY = turn;
        double rightRotationX = 0;
        double rightRotationY = -turn;

        // 3. Combine Translation (Left Stick) and Rotation (Right Stick)
        double combinedLeftX = lx + leftRotationX;
        double combinedLeftY = ly + leftRotationY;
        double combinedRightX = lx + rightRotationX;
        double combinedRightY = ly + rightRotationY;

        // 4. Calculate Individual Magnitudes and Angles
        double magLeft = Math.hypot(combinedLeftX, combinedLeftY);
        double magRight = Math.hypot(combinedRightX, combinedRightY);

        double targetAngleLeft = Math.toDegrees(Math.atan2(combinedLeftX, combinedLeftY));
        double targetAngleRight = Math.toDegrees(Math.atan2(combinedRightX, combinedRightY));

        if (targetAngleLeft < 0) targetAngleLeft += 360;
        if (targetAngleRight < 0) targetAngleRight += 360;
        //get current encoder positions
        int leftCurrentTicks = podLeft.getMotorAngle();
        int rightCurrentTicks = podRight.getMotorAngle();
        // Convert to degrees
        int leftCurrentDegrees = (int)((leftCurrentTicks / (double)Constants.GOBILDA_ENCODER_TICKS) * 360);
        int rightCurrentDegrees = (int)((rightCurrentTicks / (double)Constants.GOBILDA_ENCODER_TICKS) * 360);

        // 5. Optimization for LEFT Pod
        double driveMultLeft = 1.0;
        double diffLeft = closestAngle(targetAngleLeft, leftCurrentDegrees); //here closest angle for turning the wheel is calculated
        if (abs(diffLeft) > 90) {                                       //if the closest angle is > 90 deg
            driveMultLeft = -1.0;                                            //reverses the direction of the motor using "driveMultLeft"
            targetAngleLeft = (targetAngleLeft + 180) % 360;                 //adjusts the angle to the opposite angle
        }

        // 6. Optimization for RIGHT Pod
        double driveMultRight = 1.0;
        double diffRight = closestAngle(targetAngleRight, rightCurrentDegrees);
        if (abs(diffRight) > 90) {
            driveMultRight = -1.0;
            targetAngleRight = (targetAngleRight + 180) % 360;
        }

        // 7. Calculate Final Motor Targets
        double moveLeft = closestAngle(targetAngleLeft, leftCurrentDegrees) + leftCurrentDegrees;
        double moveRight = closestAngle(targetAngleRight, rightCurrentDegrees) + rightCurrentDegrees;

        podLeft.setAngle((int)((moveLeft / 360.0) * Constants.GOBILDA_ENCODER_TICKS)); //set angle needs motor ticks
        podRight.setAngle((int)((moveRight / 360.0) * Constants.GOBILDA_ENCODER_TICKS));


        // 8. Apply Final Drive Power
        // Normalize magnitudes if they exceed 1.0
        double maxMag = Math.max(1.0, Math.max(magLeft, magRight));

        // Determine the speed multiplier based on the toggle state
        double speedLimit = 1.0;

        // Motor power combination of left and right sides and fast/slow state
        telemetry.addData("Motor stuff", Range.clip((magLeft / maxMag) * driveMultLeft * speedLimit, -speedLimit, speedLimit));
        podLeft.setDrive(Range.clip((magLeft / maxMag) * driveMultLeft * speedLimit, -speedLimit, speedLimit));
        podRight.setDrive(Range.clip((magRight / maxMag) * driveMultRight * speedLimit, -speedLimit, speedLimit));
    }

    public void handleFlywheel(Gamepad gamepad1) {
        if (gamepad1.dpadUpWasPressed()) {
            targetRPM += 25;
        }
        if (gamepad1.dpadDownWasPressed()) {
            targetRPM -= 25;
        }

        if (gamepad1.dpadRightWasPressed()) {
            targetRPM += 250;
        }
        if (gamepad1.dpadLeftWasPressed()) {
            targetRPM -= 250;
        }

        if (gamepad1.aWasPressed()) {
            targetRPM = 0;
        }

        if (gamepad1.rightBumperWasPressed()) {
            targetRPM = (int)Constants.FLYWHEEL_DEFAULT;
        }

        double targetTPS = ((targetRPM) / 60.0) * Constants.TICKS_PER_REV;
        voltageCompensatedFlywheel.setTargetVelocity(targetTPS);
        voltageCompensatedFlywheel.update();
    }

    public void setFlywheelTarget(int target) {
        targetRPM = target;
    }

    private double closestAngle(double target, double current) { //method inputs current and target encoder position in degrees
        return (target - current + 540) % 360 - 180;
    }

    public void setGate(double pos) {
        gate.setPosition(pos);
    }
    public void setIntakePower(double pow) {
        intake.setPower(pow);
    }

    public GoBildaPinpointDriver getOdo() { return odo;}
}
