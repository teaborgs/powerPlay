package org.firstinspires.ftc.teamcode.drive.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.LinkedList;
import java.util.Queue;

@TeleOp(group = "Driving")
public class teleop2 extends LinearOpMode
{
	SampleMecanumDrive mecanumDrive;
	DcMotorEx liftMotor1, liftMotor2, wormMotor;
	Servo catcher;
	double suppress1;
	double suppressRotate;

	ElapsedTime runtime = new ElapsedTime();

	@Override
	public void runOpMode() throws InterruptedException {
		initialization();
		waitForStart();
		while (opModeIsActive() && !isStopRequested()) {
			run();
		}
	}

	private void initialization() {
		liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
		liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
		wormMotor = hardwareMap.get(DcMotorEx.class, "wormMotor");
		catcher = hardwareMap.get(Servo.class, "catcherServo");
		catcher.setPosition(0);
		liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		wormMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		wormMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		wormMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		mecanumDrive = new SampleMecanumDrive(hardwareMap);
		mecanumDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		mecanumDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
	}

	private void controlWheels() {
		Pose2d poseEstimate = mecanumDrive.getPoseEstimate();
		Vector2d input = new Vector2d(
				-gamepad1.left_stick_y * suppress1,
				-gamepad1.left_stick_x * suppress1
		).rotated(-poseEstimate.getHeading());
		mecanumDrive.setWeightedDrivePower(
				new Pose2d(
						input.getX(),
						input.getY(),
						-(gamepad1.right_trigger - gamepad1.left_trigger) * suppressRotate
				)
		);
	}

	private void resetArmLocalization() {
		if (gamepad2.right_stick_button) {
			liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
			liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		}
	}

	boolean lastPressedCatch = false;
	ElapsedTime timpCon = new ElapsedTime();
	private void controlCatcher() {
		executeCurrentMoveTarget();
		MoveTarget currentTarget;
		boolean left_bumper1_pressed = gamepad1.left_bumper;
		if (left_bumper1_pressed && !lastPressedCatch) {
			if (catcher.getPosition() == 0) {
				catcher.setPosition(.6f);
			} else {
				catcher.setPosition(0);
				resetTargets();
				if (down) {
					currentTarget = new MoveTarget(liftMotor1, -300);
					moveTargets.add(currentTarget);
					down = false;
				}
			}
		}
		lastPressedCatch = left_bumper1_pressed;
	}


	private void suppressWheels() {
		if (gamepad1.right_bumper) {
			suppress1 = 0.5f;
			suppressRotate = 0.5f;
		} else {
			suppress1 = 1f;
			suppressRotate = 1f;
		}
	}

	private void run() {
		resetArmLocalization();
		suppressWheels();
		controlWheels();
		controlArm();
		controlCatcher();
		debugTelemetry();
		//autonomousArm();
		relax();
	}

	private void debugTelemetry() {
		telemetry.addData("lift1", liftMotor1.getCurrentPosition());
		telemetry.addData("lift2", liftMotor2.getCurrentPosition());
		telemetry.addData("plate", wormMotor.getCurrentPosition());
		telemetry.addData("claw", catcher.getPosition());
		telemetry.addData("motorPower1", liftMotor1.getPower());
		telemetry.addData("motorBusy1", liftMotor1.isBusy());
		telemetry.addData("motorPower2", liftMotor2.getPower());
		telemetry.addData("motorBusy2", liftMotor2.isBusy());
		telemetry.addData("plateBusy1", wormMotor.isBusy());
		telemetry.addData("platePower", wormMotor.getPower());
		telemetry.addData("timpCon", timpCon.milliseconds());
		telemetry.update();
	}

	private class MoveTarget {
		private DcMotorEx motor;
		private int position;

		public MoveTarget(DcMotorEx motor, int position) {
			this.motor = motor;
			this.position = position;
		}

		public DcMotorEx getMotor() {
			return motor;
		}

		public int getPosition() {
			return position;
		}
	}

	boolean down = false;

	private void executeCurrentMoveTarget() {
		if (moveTargets.isEmpty()) return;

		MoveTarget moveTarget = moveTargets.peek();
		DcMotorEx motor = moveTarget.getMotor();

		if (motor == liftMotor1) {
			DcMotorEx motor2 = liftMotor2;
			motor.setTargetPosition(moveTarget.getPosition());
			motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			motor.setPower(1);
			motor2.setTargetPosition(moveTarget.getPosition());
			motor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			motor2.setPower(1);
		} else {
			motor.setTargetPosition(moveTarget.getPosition());
			motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
			motor.setPower(1);
		}

		if (runtime.milliseconds() > 200) {//800
			moveTargets.remove();
			runtime.reset();
		}
	}

	Queue<MoveTarget> moveTargets = new LinkedList<>();

	private void resetTargets() {
		moveTargets.clear();
		runtime.reset();
	}

	private void controlArm() {
		executeCurrentMoveTarget();
		MoveTarget currentTarget;
		if(gamepad2.right_trigger!=0) {
			resetTargets();
			down = false;
			currentTarget = new MoveTarget(wormMotor, 0);
			moveTargets.add(currentTarget);
			currentTarget = new MoveTarget(liftMotor1, -148);
			moveTargets.add(currentTarget);
		}
		else if(gamepad2.left_trigger!=0) {
			resetTargets();
			down = false;
			currentTarget = new MoveTarget(wormMotor, 0);
			moveTargets.add(currentTarget);
			currentTarget = new MoveTarget(liftMotor1, -89);
			moveTargets.add(currentTarget);
		}
		else if (gamepad2.a) {
			resetTargets();
			down = false;
			currentTarget = new MoveTarget(wormMotor, 0);
			moveTargets.add(currentTarget);
			currentTarget = new MoveTarget(liftMotor1, 0);
			moveTargets.add(currentTarget);
		} else if (gamepad2.dpad_left) {
			resetTargets();
			down = false;
			currentTarget = new MoveTarget(liftMotor1, -820);
			moveTargets.add(currentTarget);
			currentTarget = new MoveTarget(wormMotor, -725);
			moveTargets.add(currentTarget);
		} else if (gamepad2.dpad_right) {
			resetTargets();
			down = false;
			currentTarget = new MoveTarget(liftMotor1, -500);
			moveTargets.add(currentTarget);
			currentTarget = new MoveTarget(wormMotor, -725);
			moveTargets.add(currentTarget);
		} else if (gamepad2.dpad_up) {
			resetTargets();
			down = false;
			currentTarget = new MoveTarget(liftMotor1, -1170);
			moveTargets.add(currentTarget);
			currentTarget = new MoveTarget(wormMotor, -725);
			moveTargets.add(currentTarget);
		} else if (gamepad2.left_bumper) {
			resetTargets();
			down = true;
			currentTarget = new MoveTarget(liftMotor1, -50);
			moveTargets.add(currentTarget);
			currentTarget = new MoveTarget(wormMotor, -725);
			moveTargets.add(currentTarget);
		} else if (gamepad2.x) {
			resetTargets();
			down = false;
			currentTarget = new MoveTarget(liftMotor1, -820);
			moveTargets.add(currentTarget);
			currentTarget = new MoveTarget(wormMotor, 725);
			moveTargets.add(currentTarget);
		} else if (gamepad2.dpad_down) {
			resetTargets();
			down = false;
			currentTarget = new MoveTarget(wormMotor, 0);
			moveTargets.add(currentTarget);
			currentTarget = new MoveTarget(liftMotor1, 0);
			moveTargets.add(currentTarget);
		} else if (gamepad2.b) {
			resetTargets();
			down = false;
			currentTarget = new MoveTarget(liftMotor1, -500);
			moveTargets.add(currentTarget);
			currentTarget = new MoveTarget(wormMotor, 725);
			moveTargets.add(currentTarget);
		} else if (gamepad2.y) {
			resetTargets();
			down = false;
			currentTarget = new MoveTarget(liftMotor1, -1170);
			moveTargets.add(currentTarget);
			currentTarget = new MoveTarget(wormMotor, 725);
			moveTargets.add(currentTarget);
		} else if (gamepad2.right_bumper) {
			resetTargets();
			down = true;
			currentTarget = new MoveTarget(liftMotor1, -50);
			moveTargets.add(currentTarget);
			currentTarget = new MoveTarget(wormMotor, 725);
			moveTargets.add(currentTarget);
		}
	}

	private void relax() {
		if (moveTargets.isEmpty() && runtime.milliseconds() > 1000) {
			wormMotor.setPower(0);
		}
		if ((liftMotor2.getCurrentPosition() >= -350 || liftMotor1.getCurrentPosition() >= -350)
				&& moveTargets.isEmpty() && runtime.milliseconds() > 1000) {
			liftMotor1.setPower(0);
			liftMotor2.setPower(0);
		} else if ((liftMotor2.getCurrentPosition() >= -800 || liftMotor1.getCurrentPosition() >= -800) && moveTargets.isEmpty() && runtime.milliseconds() > 1000) {
			liftMotor1.setPower(0.05f);
			liftMotor2.setPower(0.05f);
		} else if(moveTargets.isEmpty() && runtime.milliseconds() > 1000){
			liftMotor1.setPower(0.1f);
			liftMotor2.setPower(0.1f);
		}
	}
}
