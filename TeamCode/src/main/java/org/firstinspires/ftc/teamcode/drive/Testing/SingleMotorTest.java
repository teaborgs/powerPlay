package org.firstinspires.ftc.teamcode.drive.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(group = "Testing")
public class SingleMotorTest extends LinearOpMode
{
	DcMotorEx motor;

	@Override
	public void runOpMode() throws InterruptedException
	{
		Init();
		waitForStart();

		while (opModeIsActive() && !isStopRequested())
			RunLoop();
	}

	private void Init()
	{
		motor = hardwareMap.get(DcMotorEx.class, "slot0");
		motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	private void RunLoop()
	{
		motor.setPower(gamepad1.right_stick_y);
	}
}