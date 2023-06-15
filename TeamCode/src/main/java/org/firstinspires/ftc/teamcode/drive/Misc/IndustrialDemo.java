package org.firstinspires.ftc.teamcode.drive.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInputController;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.configuration.annotations.AnalogSensorType;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorREV2mDistance;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@AnalogSensorType
@DeviceProperties(name = "@string/configTypeAnalogInput", xmlTag = "AnalogInputCustom", builtIn = false)
class AnalogInputCustom implements HardwareDevice
{
    public enum State
    {
        None("None"),
        Idle("Idle"),
        Detection1("Det 1"),
        Detection2("Det 2"),
        Detection3("Det 3");

        public final String name;

        State(String _name)
        {
            this.name = _name;
        }
    }

    public State GetState()
    {
        double reading = this.getVoltage();

        // cele mai usor de detectat
        if (reading < 0.1)
            return State.Idle;
        if (reading > 4.8)
            return State.Detection3;

        // pot fii ambigue
        if (reading < 1.5)
            return State.Detection1;
        if (reading < 3.5)
            return State.Detection2;

        return State.None;
    }

    private AnalogInputController controller = null;
    private int channel = -1;

    public AnalogInputCustom(AnalogInputController controller, int channel) {
        this.controller = controller;
        this.channel = channel;
    }

    @Override public Manufacturer getManufacturer() {
        return controller.getManufacturer();
    }

    public double getVoltage() {
        return controller.getAnalogInputVoltage(channel);
    }

    public double getMaxVoltage() {
        return controller.getMaxAnalogInputVoltage();
    }

    @Override
    public String getDeviceName() {
        return AppUtil.getDefContext().getString(R.string.configTypeAnalogInput);
    }

    @Override
    public String getConnectionInfo() {
        return controller.getConnectionInfo() + "; analog port " + channel;
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
    }

    @Override
    public void close() {
        // take no action
    }
}

@Autonomous(group = "autonom")
public class IndustrialDemo extends LinearOpMode
{
    SampleMecanumDrive mecanumDrive;
    AnalogInputCustom in;
    SensorREV2mDistance

    @Override
    public void runOpMode() throws InterruptedException
    {
        Init();

        waitForStart();

        while (!isStopRequested())
            Run();
    }

    private void Init()
    {
 //       mecanumDrive = new SampleMecanumDrive(hardwareMap);
        in = hardwareMap.get(AnalogInputCustom.class, "analogIn");

        telemetry.setMsTransmissionInterval(100);
    }

    private void Run()
    {
        Telemetry();
    }

    private void Telemetry()
    {
        telemetry.addLine("Reading");
    //    telemetry.addData("I2C Reading", arduino.GetDetection());
        telemetry.addData("Analog Reading", in.getVoltage());
        telemetry.addData("Analog Reading", in.GetState());
        telemetry.update();
    }
}