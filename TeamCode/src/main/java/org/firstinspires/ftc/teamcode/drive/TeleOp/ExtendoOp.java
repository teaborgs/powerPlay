package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.OdometryMecanumDrive;


@TeleOp(group = "Driving")
public final class ExtendoOp extends LinearOpMode
{
	OdometryMecanumDrive mecanumDrive;

	DcMotorEx extendo1, extendo2, lift, worm;

	Servo claw, loader, scorer;


	private static final int LiftUp = 1130;
	private static final int LiftDown = 80;
	private static final float LiftPower = 0.66f;

	private static final float ClawOpen = 1.0f;
	private static final float ClawClosed = 0.0f;

	private static final float LoaderUp = 1.0f;
	private static final float LoaderDown = 0.0f;



	@Override
	public void runOpMode() throws InterruptedException
	{
		Init();

		waitForStart();

		while (opModeIsActive() && !isStopRequested())
			Run();
	}


	private void Run()
	{
		Drive();
		Arms();
		Telemetry();
	}


	private void Drive()
	{
		// Suppress
		float suppress = 1.0f;
		if (gamepad1.left_bumper)
			suppress = 0.5f;

		// Control
		Vector2d in = new Vector2d(gamepad1.right_stick_x * suppress, gamepad1.right_stick_y * suppress)
					   .rotated(-mecanumDrive.getPoseEstimate().getHeading());
		mecanumDrive.setWeightedDrivePower(new Pose2d(in.getX(), in.getY(), gamepad1.left_trigger - gamepad1.right_trigger));
	}


	private void Arms()
	{
		Extendo();
		Worm();
		Lift();
		Claw();
		Loader();
	}

	private void Worm()
	{
		worm.setPower(gamepad2.left_trigger - gamepad2.right_trigger);
	}


	boolean extendedLift = false;
	boolean safe2 = false;
	private void Lift()
	{
		// Input
		if (gamepad1.right_bumper && !safe2)
		{
			extendedLift = !extendedLift;

			safe2 = true;
		}
		else if (!gamepad1.right_bumper)
			safe2 = false;


		// Move
		lift.setTargetPosition(extendedLift ? LiftUp : LiftDown);
		lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		lift.setPower(LiftPower);
	}

	private void Extendo()
	{
		extendo1.setPower(gamepad2.right_stick_y);
		extendo2.setPower(gamepad2.right_stick_y);
	}


	private boolean clawOpen = true;
	private boolean safe = false;
	private void Claw()
	{
		// Input
		if (gamepad1.right_bumper && !safe)
		{
			clawOpen = !clawOpen;

			safe = true;
		}
		else if (!gamepad1.right_bumper)
			safe = false;

		// Move
		claw.setPosition(clawOpen ? ClawOpen : ClawClosed);
	}

	private boolean loaderUp = false;
	private boolean safe3 = false;
	private void Loader()
	{
		// Input
		if (gamepad2.x && !safe3)
		{
			loaderUp = !loaderUp;

			safe3 = true;
		}
		else if (!gamepad1.right_bumper)
			safe3 = false;

		// Move
		loader.setPosition(loaderUp ? LoaderUp : LoaderDown);
	}

	private void Telemetry()
	{
		telemetry.addData("worm pos", worm.getCurrentPosition());
		telemetry.addData("worm pow", worm.getPower());
		telemetry.addData("lift pos", lift.getCurrentPosition());
		telemetry.addData("lift pow", lift.getPower());
		telemetry.addData("extendo1 pos", extendo1.getCurrentPosition());
		telemetry.addData("extendo2 pos", extendo2.getCurrentPosition());
		telemetry.addData("extendo1 pow", extendo1.getPower());
		telemetry.addData("extendo2 pow", extendo2.getPower());
		telemetry.addData("position", mecanumDrive.getPoseEstimate());
		telemetry.addData("velocity", mecanumDrive.getPoseVelocity());
		telemetry.update();
	}


	private void Init()
	{
		// Chassis
		mecanumDrive = new OdometryMecanumDrive(hardwareMap);
		mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		mecanumDrive.setPoseEstimate(new Pose2d(0, 0));

		// Worm
		worm = hardwareMap.get(DcMotorEx.class, "slot4");
		worm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		worm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		worm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		// Lift
		lift = hardwareMap.get(DcMotorEx.class, "slot5");
		lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		lift.setTargetPosition(LiftDown);
		lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		lift.setPower(LiftPower);

		// Extendo
		extendo1 = hardwareMap.get(DcMotorEx.class, "slot6");
		extendo1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		extendo1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		extendo1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		extendo2 = hardwareMap.get(DcMotorEx.class, "slot7");
		extendo2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		extendo2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		extendo2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		extendo2.setDirection(DcMotorSimple.Direction.REVERSE);

		// Servos
		claw = hardwareMap.get(Servo.class, "claw");
		claw.setPosition(0.0f);

		loader = hardwareMap.get(Servo.class, "loader");
		loader.setPosition(0.0f);

		scorer = hardwareMap.get(Servo.class, "scorer");
		scorer.setPosition(0.0f);
	}
}