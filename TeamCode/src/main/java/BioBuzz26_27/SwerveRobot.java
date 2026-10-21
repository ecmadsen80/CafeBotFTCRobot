package BioBuzz26_27;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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

    public void driveTo(double direction, double amount, double speed) {
        double changeX = amount * Math.cos(Math.toRadians(direction));
        double changeY = amount * Math.sin(Math.toRadians(direction));
    }

    //public void drive();

}
