package org.firstinspires.ftc.teamcode.drive.Misc;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;
import com.qualcomm.robotcore.util.TypeConversion;


@SuppressWarnings({"WeakerAccess", "unused"})
@I2cDeviceType
@DeviceProperties(name = "Arduino Device", description = "an Arduino Device", xmlTag = "arduino")
public class ArduinoDevice extends I2cDeviceSynchDevice<I2cDeviceSynch>
{
    ////////////////////////////////////////////////////////////////////////////////////////////////
    // User Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public short GetDetection()
    {
        return GetDetectionRaw();
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Raw Register Reads
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public short GetDetectionRaw()
    {
        return readShort(Register.DETECTION);
    }

    public short getManufacturerIDRaw()
    {
        return readShort(Register.MANUFACTURER_ID);
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Read and Write Methods
    ////////////////////////////////////////////////////////////////////////////////////////////////

    protected void writeShort(final Register reg, short value)
    {
        deviceClient.write(reg.bVal, TypeConversion.shortToByteArray(value));
    }

    protected short readShort(Register reg)
    {
        return TypeConversion.byteArrayToShort(deviceClient.read(reg.bVal, 2));
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Registers and Config Settings
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public enum Register
    {
        FIRST(0),
        CONFIGURATION(0x01),
        DETECTION(0x02),
        T_LIMIT_LOWER(0x03),
        T_LIMIT_CRITICAL(0x04),
        T_LIMIT_UPPER(0x05),
        MANUFACTURER_ID(0x06),
        DEVICE_ID_REVISION(0x07),
        RESOLUTION(0x08),
        LAST(RESOLUTION.bVal);

        public int bVal;

        Register(int bVal)
        {
            this.bVal = bVal;
        }
    }

    public enum Hysteresis
    {
        HYST_0(0x0000),
        HYST_1_5(0x0200),
        HYST_3(0x0400),
        HYST_6(0x0600);

        public int bVal;

        Hysteresis(int bVal)
        {
            this.bVal = bVal;
        }
    }

    public enum AlertControl
    {
        ALERT_DISABLE(0x0000),
        ALERT_ENABLE(0x0008);

        public int bVal;

        AlertControl(int bVal)
        {
            this.bVal = bVal;
        }
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////
    // Construction and Initialization
    ////////////////////////////////////////////////////////////////////////////////////////////////

    public final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x18);

    public ArduinoDevice(I2cDeviceSynch deviceClient)
    {
        super(deviceClient, true);

        this.setOptimalReadWindow();
        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);

        super.registerArmingStateCallback(false); // Deals with USB cables getting unplugged
        // Sensor starts off disengaged so we can change things like I2C address. Need to engage
        this.deviceClient.engage();
    }

    protected void setOptimalReadWindow()
    {
        // Sensor registers are read repeatedly and stored in a register. This method specifies the
        // registers and repeat read mode
        I2cDeviceSynch.ReadWindow readWindow = new I2cDeviceSynch.ReadWindow(
                Register.FIRST.bVal,
                Register.LAST.bVal - Register.FIRST.bVal + 1,
                I2cDeviceSynch.ReadMode.REPEAT);
        this.deviceClient.setReadWindow(readWindow);
    }

    @Override
    protected synchronized boolean doInitialize()
    {
        int configSettings = Hysteresis.HYST_1_5.bVal | AlertControl.ALERT_ENABLE.bVal;

        writeShort(Register.CONFIGURATION, (short) configSettings);

        // Mask out alert signal bit, which we can't control
        return (readShort(Register.CONFIGURATION) & 0xFFEF) == configSettings;
    }

    @Override
    public Manufacturer getManufacturer()
    {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName()
    {
        return "Arduino Device";
    }
}