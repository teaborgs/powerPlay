package org.firstinspires.ftc.teamcode.drive.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "Testing")
public class Test extends LinearOpMode
{
	SampleMecanumDrive mecanumDrive;

	@Override
	public void runOpMode() throws InterruptedException
	{
		Init();
		waitForStart();
		while (!isStopRequested() && opModeIsActive())
			Run();
	}

	void Init()
	{
		mecanumDrive = new SampleMecanumDrive(hardwareMap);
		mecanumDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		telemetry.setMsTransmissionInterval(50);
	}

	void Run()
	{
		Telemetry();
	}


	void Telemetry()
	{
		telemetry.addData("Heading", mecanumDrive.getExternalHeadingVelocity());
		telemetry.update();
	}
}
