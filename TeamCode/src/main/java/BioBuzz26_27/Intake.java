package BioBuzz26_27;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intake;
    private double power;
    public void init(HardwareMap hw) {
        intake = hw.get(DcMotor.class, "intake");
        power = 0.0;
    }

    public void setPower(double p) {
        intake.setPower(power);
    }

}
