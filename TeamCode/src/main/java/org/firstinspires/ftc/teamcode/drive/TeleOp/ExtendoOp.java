package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.teamcode.drive.OdometryMecanumDrive;

@TeleOp(group = "Driving")
public class ExtendoOp extends LinearOpMode
{
	OdometryMecanumDrive mecanumDrive;

	DcMotorEx extendo1, extendo2, lift;

	@Override
	public void runOpMode() throws InterruptedException
	{
		Init();

		waitForStart();

		while (opModeIsActive() && !isStopRequested())
		{
			Run();
		}
	}


	private void Run()
	{
		Drive();
		Extendo();
		Lift();
		Telemetry();
	}


	private void Drive()
	{
		mecanumDrive.setWeightedDrivePower(new Pose2d(gamepad1.right_stick_x, gamepad1.right_stick_y));
	}

	private void Lift()
	{
		lift.setPower(gamepad2.left_stick_y);
	}

	private void Extendo()
	{
		extendo1.setPower(gamepad2.right_stick_y);
		extendo2.setPower(gamepad2.right_stick_y);
	}

	private void Telemetry()
	{
		telemetry.update();
	}


	private void Init()
	{
		mecanumDrive = new OdometryMecanumDrive(hardwareMap);
		mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		mecanumDrive.setPoseEstimate(new Pose2d(0, 0));

		lift = hardwareMap.get(DcMotorEx.class, "slot5");
		lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		lift.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		extendo1 = hardwareMap.get(DcMotorEx.class, "slot6");
		extendo1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		extendo1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		extendo1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		extendo2 = hardwareMap.get(DcMotorEx.class, "slot7");
		extendo2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		extendo2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		extendo2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		extendo2.setDirection(DcMotorSimple.Direction.REVERSE);
	}
}
