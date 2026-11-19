package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxI2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
@I2cDeviceType
@DeviceProperties(
  name = "Adafruit Absolute Encoder",
  xmlTag = "AdafruitAbsoluteEncoder",
  description = "Adafruit AS5600 Absolute Encoder"
  )
public class AbsoluteEncoderDriver extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {
  private int ticks = 0;
  public static final byte DEFAULT_ADDR = 0x36;
  public static final byte ANGLE_REG = 0x0E;

  public AbsoluteEncoderDriver(I2cDeviceSynchSimple deviceClient, boolean deviceClientIsOwned) {
    super(deviceClient, deviceClientIsOwned);
    this.deviceClient.setI2cAddress(I2cAddr.create7bit(DEFAULT_ADDR));
    super.registerArmingStateCallback(false);
  }

  @Override
  public Manufacturer getManufacturer() {
    return Manufacturer.Adafruit;
  }

  @Override
  protected synchronized boolean doInitialize() {
    ((LynxI2cDeviceSynch) (deviceClient)).setBusSpeed(LynxI2cDeviceSynch.BusSpeed.FAST_400K);
    return true;
  }

  @Override
  public String getDeviceName() {
    return "Adafruit AS5600 Absolute Encoder";
  }

  public int getTicks() {
    return ticks;
  }

  public double getAngle() {
    return (ticks * 360) / 4096.0;
  }

  public void update() {
    byte[] angleBytes = deviceClient.read(ANGLE_REG, 2);

    int high = angleBytes[0] & 0xFF;
    int low = angleBytes[1] & 0xFF;

    ticks = ((high << 8) | low) & 0x0FFF;
  }
}