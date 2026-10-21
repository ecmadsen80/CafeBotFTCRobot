package BioBuzz26_27;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class SwervePod {
    private DcMotorEx turn;
    private DcMotor drive;
    private AbsoluteEncoder encoder;

    public void init(HardwareMap hw, String t, String d, String e) {
        turn = hw.get(DcMotorEx.class, t);
        drive = hw.get(DcMotor.class, d);
        encoder.init(hw, e);
        calibrate();
    }

    public void zero(double zeroAngle) {
        turn.setPower(1.0);
        double currentAngle = encoder.getAngle();
        double angleError = ((zeroAngle - currentAngle + 540) % 360) - 180;
        int deltaTicks = (int) Math.round(angleError / 360.0 * 751.8);

        turn.setTargetPosition(turn.getCurrentPosition() - deltaTicks);

        turn.setVelocity(2500);
    }

    public boolean isBusy() {return turn.isBusy();}
    public void setDrive(double power) {
        drive.setPower(0.0);
    }
    public void calibrate() {
        turn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turn.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turn.setTargetPosition(0);
        turn.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
