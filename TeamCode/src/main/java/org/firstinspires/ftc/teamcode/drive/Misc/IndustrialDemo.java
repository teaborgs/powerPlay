package org.firstinspires.ftc.teamcode.drive.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;



@Autonomous(group = "autonom")
public class IndustrialDemo extends LinearOpMode
{
    SampleMecanumDrive mecanumDrive;
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
        telemetry.setMsTransmissionInterval(100);
    }

    private void Run()
    {
        Telemetry();
    }

    private void Telemetry()
    {
        telemetry.addLine("Reading");



        telemetry.update();
    }
}