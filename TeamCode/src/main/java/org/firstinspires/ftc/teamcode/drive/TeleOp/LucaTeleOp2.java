package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "LucaTeleOp2", group = "Driving")
public class LucaTeleOp2 extends LinearOpMode
{
	SampleMecanumDrive mecanumDrive;

	Gamepad wheelGamepad, armGamepad;

	DcMotorEx intakeMotor;
	DcMotorEx liftMotor1, liftMotor2;
	DcMotorEx tumblerMotor;

	int IdleTumblerPos = 200;
	int BoardTumblerPos = 450;
	int LoadTumblerPos = 0;

	Servo clawServo, lockerServo, rotatorServo;

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
		mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

		intakeMotor = hardwareMap.get(DcMotorEx.class, "slot2");
		intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

		liftMotor1 = hardwareMap.get(DcMotorEx.class, "slot3");
		liftMotor2 = hardwareMap.get(DcMotorEx.class, "slot7");
		liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		liftMotor2.setDirection(DcMotorSimple.Direction.REVERSE);

		tumblerMotor = hardwareMap.get(DcMotorEx.class, "slot");
		tumblerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
		tumblerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		tumblerMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		tumblerMotor.setTargetPosition(0);
		tumblerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


		clawServo = hardwareMap.get(Servo.class, "servo0");
		lockerServo = hardwareMap.get(Servo.class, "servo1");
		rotatorServo = hardwareMap.get(Servo.class, "servo2");

		wheelGamepad = gamepad1;
		armGamepad = gamepad2;

		telemetry.setMsTransmissionInterval(50);
	}

	private void Run()
	{
		Wheels();
		Intake();
		Lift();
		Claw();
		Tumbler();
		Rotator();
		Locker();
		Telemetry();
	}

	private void Tumbler()
	{
		if(armGamepad.dpad_up)
			tumblerMotor.setTargetPosition(BoardTumblerPos);
		else if(armGamepad.dpad_down)
			tumblerMotor.setTargetPosition(LoadTumblerPos);
		else if(armGamepad.dpad_left)
			tumblerMotor.setTargetPosition(IdleTumblerPos);
		tumblerMotor.setPower(0.5);
		tumblerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
	}

	private boolean safe0 = false;
	private double lastServoPos0 = 0.0;
	private void Claw()
	{
		if (armGamepad.a && !safe0)
		{
			clawServo.setPosition(lastServoPos0 == 0.0 ? (lastServoPos0 = 0.5) : (lastServoPos0 = 0.0));
			safe0 = true;
		}
		else safe0 = armGamepad.a;
	}

	private boolean safe1 = false;
	private double lastServoPos1 = 0.0;
	private void Rotator()
	{
		if(armGamepad.b && !safe1 && tumblerMotor.getCurrentPosition() > IdleTumblerPos)
		{
			rotatorServo.setPosition(lastServoPos1 == 0.0 ? (lastServoPos1 = 1.0) : (lastServoPos1 = 0.0));
			safe1 = true;
		}
		else safe1 = armGamepad.b;
	}

	private boolean safe2 = false;
	private double lastServoPos2 = 0.0;
	private void Locker()
	{
		if(armGamepad.y && !safe2)
		{
			lockerServo.setPosition(lastServoPos2 == 0.0 ? (lastServoPos2 = 1.0) : (lastServoPos2 = 0.2));
			safe2 = true;
		}
		else safe2 = armGamepad.y;
	}

	private void Lift()
	{
		double power = armGamepad.right_stick_y;

		liftMotor1.setPower(power);
		liftMotor2.setPower(power);
	}

	private void Intake()
	{
//		if (wheelGamepad.left_bumper)
//			intakeMotor.setPower(1.0);
//		else
//			intakeMotor.setPower(0.0);
		intakeMotor.setPower(wheelGamepad.right_stick_y);
	}

	private void Wheels()
	{
		// Suppress
		double suppressVal = 1.0;
		if (wheelGamepad.right_bumper)
			suppressVal = 0.6;

		// Control
		Vector2d input = new Vector2d(
				wheelGamepad.left_stick_y * suppressVal,
				wheelGamepad.left_stick_x * suppressVal
		).rotated(-mecanumDrive.getPoseEstimate().getHeading());
		mecanumDrive.setDrivePower(
				new Pose2d(
						input.getX(),
						input.getY(),
						-(wheelGamepad.right_trigger - wheelGamepad.left_trigger) * suppressVal
				)
		);
	}

	private void Telemetry()
	{
		telemetry.addData("Intake Power: ", intakeMotor.getPower());

		telemetry.addData("Lift1 Power: ", liftMotor1.getPower());
		telemetry.addData("Lift2 Power: ", liftMotor2.getPower());
		telemetry.addData("Lift2 Pos: ", liftMotor2.getCurrentPosition());
		telemetry.addData("Lift2 Pos: ", liftMotor2.getCurrentPosition());

		telemetry.addData("Tumbler Power: ", tumblerMotor.getPower());
		telemetry.addData("Tumbler Pos: ", tumblerMotor.getCurrentPosition());
		telemetry.addData("Tumbler Target Pos: ", tumblerMotor.getTargetPosition());

		telemetry.update();
	}
}