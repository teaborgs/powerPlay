package org.firstinspires.ftc.teamcode.drive.Misc;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.Misc.ArduinoDevice;

import org.firstinspires.ftc.robotcore.internal.hardware.TimeWindow;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.concurrent.locks.Lock;

public class IndustrialDemo extends LinearOpMode
{
    SampleMecanumDrive mecanumDrive;
    ArduinoDevice arduino;

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
        mecanumDrive = new SampleMecanumDrive(hardwareMap);

    }

    private void Run()
    {
        Telemetry();
    }

    private void Telemetry()
    {
        telemetry.addData("I2C Reading", arduino.GetDetection());
    }
}