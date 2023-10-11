package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(name = "LucaTeleOp2", group = "Driving")
public class LucaTeleOp2 extends LinearOpMode
{
	SampleMecanumDrive mecanumDrive;

	Gamepad wheelGamepad, armGamepad;

	DcMotorEx intakeMotor;
	DcMotorEx liftMotor1, liftMotor2;

	Servo clawServo, tumblerServo1, tumblerServo2, backflipperServo;

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

		liftMotor1 = hardwareMap.get(DcMotorEx.class, "slot3");
		liftMotor2 = hardwareMap.get(DcMotorEx.class, "slot7");
		liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


		clawServo = hardwareMap.get(Servo.class, "servo0");

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
		Telemetry();
	}

	private boolean safe1 = false;
	private double lastTumblerPos = 0.0;
	private double lastBackflipperPos = 0.0;
	private void Tumbler()
	{
		if (armGamepad.b && !safe1)
		{
			tumblerServo1.setPosition(lastTumblerPos == 0.0 ? (lastTumblerPos = 0.7) : (lastTumblerPos = 0.0));
			tumblerServo2.setPosition(lastTumblerPos == 0.0 ? (lastTumblerPos = 0.7) : (lastTumblerPos = 0.0));
			backflipperServo.setPosition(lastBackflipperPos == 0.0 ? (lastBackflipperPos = 1.0) : (lastBackflipperPos = 0.0));
			safe1 = true;
		}
		else safe1 = armGamepad.b;
	}

	private boolean safe0 = false;
	private double lastServoPos = 0.0;
	private void Claw()
	{
		if (armGamepad.a && !safe0)
		{
			clawServo.setPosition(lastServoPos == 0.0 ? (lastServoPos = 1.0) : (lastServoPos = 0.0));
			safe0 = true;
		}
		else safe0 = armGamepad.a;
	}

	private void Lift()
	{
		double power = armGamepad.right_stick_y;

		liftMotor1.setPower(power);
		liftMotor2.setPower(power);
	}

	private void Intake()
	{
		if (wheelGamepad.left_bumper)
			intakeMotor.setPower(1.0);
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
		telemetry.addData("Input: ", gamepad1.left_stick_x + " " + wheelGamepad.left_stick_y);
		telemetry.addData("Intake Power: ", intakeMotor.getPower());
		telemetry.addData("Lift1 Power: ", liftMotor1.getPower());
		telemetry.addData("Lift2 Power: ", liftMotor2.getPower());
		telemetry.update();
	}
}