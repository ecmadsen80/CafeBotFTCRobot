package BioBuzz26_27;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

public class AbsoluteEncoder {
    private Rev2mDistanceSensor dummy;
    private I2cDeviceSynch encoder;

    public void init(HardwareMap hw, String name) {
        dummy = hw.get(Rev2mDistanceSensor.class, name);
        encoder = dummy.getDeviceClient();
        encoder.setI2cAddress(I2cAddr.create7bit(0x36));
        encoder.engage();
    }

    public double getAngle() {
        byte[] angleBytes = encoder.read(0x36, 2);

        int high = angleBytes[0] & 0xFF;
        int low = angleBytes[1] & 0xFF;

        int rawAngle = ((high << 8) | low) & 0x0FFF;
        return rawAngle * 360.0 / 4096.0;
    }
}
