package org.firstinspires.ftc.teamcode.drive.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(group = "Testing")
public final class DualInverseMotorTest extends LinearOpMode
{
	DcMotorEx motor1, motor2;

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
		motor1 = hardwareMap.get(DcMotorEx.class, "slot0");
		motor2 = hardwareMap.get(DcMotorEx.class, "slot1");
		motor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		motor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		//motor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		//motor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		motor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		motor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		telemetry.setMsTransmissionInterval(100);
	}

	private void RunLoop()
	{
		if (gamepad1.right_stick_y != 0)
		{
			motor1.setPower(Fn( gamepad1.right_stick_y));
			motor2.setPower(Fn(-gamepad1.right_stick_y));
		}//*/

	/*	motor1.setTargetPosition(350);
		motor2.setTargetPosition(350);
		motor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		motor1.setPower(1.0f);
		motor2.setPower(-1.0f);//*/

		Telemetry();
	}

	private void Telemetry()
	{
		telemetry.addData("Motor 1 power:", motor1.getPower());
		telemetry.addData("Motor 2 power:", motor2.getPower());
		telemetry.addData("Motor1 pos: ", motor1.getCurrentPosition());
		telemetry.addData("Motor2 pos: ", motor2.getCurrentPosition());
		telemetry.update();
	}

	private float Fn(float x)
	{
		float sign = Math.abs(x) / x;
		return x * x * sign;
	}
}