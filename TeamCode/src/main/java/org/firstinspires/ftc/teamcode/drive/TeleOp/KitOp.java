package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(group = "Driving")
public class KitOp extends LinearOpMode
{
	DcMotorEx motor1, motor2;

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
		motor1 = hardwareMap.get(DcMotorEx.class, "slot0");
		motor1.setDirection(DcMotorSimple.Direction.REVERSE);

		motor2 = hardwareMap.get(DcMotorEx.class, "slot1");

		telemetry.setMsTransmissionInterval(50);
	}

	private void Run()
	{
		// Wheels
		double motorPower1, motorPower2;

		motorPower1 = motorPower2 = gamepad1.right_stick_y;

		motorPower1 -= gamepad1.right_stick_x;
		motorPower2 += gamepad1.right_stick_x;

		motorPower1 = Math.min(motorPower1, 1.0); motorPower1 = Math.max(motorPower1, -1.0);
		motorPower2 = Math.min(motorPower2, 1.0); motorPower2 = Math.max(motorPower2, -1.0);

		motor1.setPower(motorPower1);
		motor2.setPower(motorPower2);


		telemetry.addData("Stick X: ", gamepad1.right_stick_x);
		telemetry.addData("Stick Y: ", gamepad1.right_stick_y);
		telemetry.addData("Motor 1 Pow: ", motorPower1);
		telemetry.addData("Motor 2 Pow: ", motorPower2);
		telemetry.update();
	}
}
