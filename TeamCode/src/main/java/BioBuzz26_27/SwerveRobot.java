package BioBuzz26_27;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import java.lang.Math;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class SwerveRobot {
    private SwervePod podLeft;
    private SwervePod podRight;
    private Intake intake;
    private GoBildaPinpointDriver odo;
    private ElapsedTime time = new ElapsedTime();

    public void init(HardwareMap hw) {
        podLeft.init(hw, "left_turn", "left_drive", "as5600Left");
        podRight.init(hw, "right_turn", "right_drive", "as5600Right");
        intake.init(hw);

        odo = hw.get(GoBildaPinpointDriver.class, "odo");

        odo.setOffsets(170, 170, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        odo.resetPosAndIMU();

        time.reset();
        while (time.time() < 0.5) {} //give IMU time to reset.
        time.reset();
        telemetry.addLine("Initialized");
        odo.update();


        odo.setPosition(new Pose2D(DistanceUnit.INCH, -54, 48, AngleUnit.DEGREES, 180)); // On blue side,
        odo.update();

        telemetry.addData(">", "Robot Ready.  Press Play.");
        telemetry.update();
    }

    public void zeroPods(double leftZero, double rightZero) {
        podLeft.zero(leftZero);
        podRight.zero(rightZero);
        telemetry.addLine("Aligning motors...");
        telemetry.update();
        while (podLeft.isBusy() || podRight.isBusy()) {}
        podLeft.calibrate();
        podRight.calibrate();
    }

    public void driveAt(double direction, double amount, double speed) {
        double changeX = amount * Math.cos(Math.toRadians(direction));
        double changeY = amount * Math.sin(Math.toRadians(direction));
    }

    public void driveGamepad(double lx, double ly) {
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
        int leftCurrentDegrees = (int)((leftCurrentTicks / (double)GOBILDA_ENCODER_TICKS) * 360);
        int rightCurrentDegrees = (int)((rightCurrentTicks / (double)GOBILDA_ENCODER_TICKS) * 360);

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

        podLeft.setAngle((int)((moveLeft / 360.0) * GOBILDA_ENCODER_TICKS));
        podLeft.setAngle((int)((moveRight / 360.0) * GOBILDA_ENCODER_TICKS));

        double maxMag = Math.max(1.0, Math.max(magLeft, magRight));
        double speedLimit = 1.0;
        podLeft.setPower(Range.clip((magLeft / maxMag) * driveMultLeft * speedLimit, -speedLimit, speedLimit));
        podRight.setPower(Range.clip((magRight / maxMag) * driveMultRight * speedLimit, -speedLimit, speedLimit));
    }

    private double closestAngle(double target, double current) { //method inputs current and target encoder position in degrees
        double dir = (target % 360) - (current % 360);           //reduces encoder angle to a value between 0 and 359
        if (abs(dir) > 180.0) {                             //If the angle is > 180 degrees
            dir = -(Math.signum(dir) * 360.0) + dir;            //subtracts the angle from 360
        }
        return dir;
    }
}
