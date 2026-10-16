package BioBuzz26_27;


import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group="BIOBUZZ", name="BIOBUZZ-Pushbot")
public class Pushbot extends LinearOpMode {

    private DcMotor leftMotor = null;
    private DcMotor rightMotor = null;
    private DcMotor flywheel = null;


    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            leftMotor.setPower(gamepad1.right_stick_y);
            rightMotor.setPower(gamepad1.left_stick_y);
            flywheel.setPower(flywheel.getPower() + ((gamepad1.dpadUpWasPressed() ? 1 : 0) * 0.05) - ((gamepad1.dpadDownWasPressed() ? 1 : 0) * 0.05));
            telemetry.addData("Flywheel: ", flywheel.getPower());
            telemetry.update();
 ;
        }

    }
}
