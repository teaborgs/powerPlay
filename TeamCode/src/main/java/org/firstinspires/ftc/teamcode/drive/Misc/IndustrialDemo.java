package org.firstinspires.ftc.teamcode.drive.Misc;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(group = "autonom")
public class IndustrialDemo extends LinearOpMode
{
	SampleMecanumDrive mecanumDrive;
	DcMotorEx liftMotor1;
	DcMotorEx liftMotor2;
	DcMotorEx wormMotor;
	Servo catcher;

	OpenCvWebcam webcam;
	CustomPipeline pipeline;
	int cameraMonitorViewId = 0;

	TrajectorySequence BluCube1, BluCubeReturn, BluSphere2, BluSphereReturn, RedCube3, RedCubeReturn, RedSphere4, RedSphereReturn, park;

	int groundLevel = -60;
	int upperLevel = -635;

	int neutralWormPos = 0;
	int leftWormPos = -745;

	@Override
	public void runOpMode() throws InterruptedException
	{
		Init();

		waitForStart();

		pipeline.SetDebug(false);

	//	while (!isStopRequested())
			Run();
	}

	private void Init()
	{
		telemetry.setMsTransmissionInterval(100);

		pipeline = new CustomPipeline();
		pipeline.SetApproximation(4);

		cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
		webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
		webcam.setMillisecondsPermissionTimeout(2500);
		webcam.setPipeline(pipeline);
		webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
		{
			@Override
			public void onOpened()
			{
				webcam.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
				ExposureControl exposure = webcam.getExposureControl();
				exposure.setMode(ExposureControl.Mode.Auto);

				GainControl gain = webcam.getGainControl();
				gain.setGain(30);
			}

			@Override
			public void onError(int errorCode)
			{
				telemetry.addData("Cam Error: ", errorCode);
				telemetry.update();
			}
		});

		mecanumDrive = new SampleMecanumDrive(hardwareMap);
		mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		mecanumDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		catcher = hardwareMap.get(Servo.class, "catcherServo");
		SetClaw(true);

		wormMotor = hardwareMap.get(DcMotorEx.class, "wormMotor");
		wormMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		wormMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		wormMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


		liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
		liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
		liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		SetLift(-60);


		BluCube1 = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
				.forward(1)
				.back(55.1)
				.strafeLeft(12)
				.addTemporalMarker(0.75, () -> SetWorm(leftWormPos))
				.addTemporalMarker(0.2, () -> SetClaw(false))
				.build();
		BluCubeReturn = mecanumDrive.trajectorySequenceBuilder(BluCube1.end())
				.strafeRight(12)
				.forward(54.1)
				.addTemporalMarker(1.0, () -> SetWorm(neutralWormPos))
				.build();


		BluSphere2 = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
				.forward(1)
				.back(55.1)
				.strafeLeft(12)
				.addTemporalMarker(0.5, () -> SetWorm(leftWormPos))
				.addTemporalMarker(0.5, () -> SetLift(upperLevel))
				.addTemporalMarker(0.2, () -> SetClaw(false))
				.build();
		BluSphereReturn = mecanumDrive.trajectorySequenceBuilder(BluSphere2.end())
				.strafeRight(12)
				.forward(54.1)
				.addTemporalMarker(0.5, () -> SetWorm(neutralWormPos))
				.addTemporalMarker(0.5, () -> SetLift(groundLevel))
				.build();



		RedCube3 = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
				.forward(1)
				.back(37.4)
				.strafeLeft(11.5)
				.addTemporalMarker(0.5, () -> SetWorm(leftWormPos))
				.addTemporalMarker(0.2, () -> SetClaw(false))
				.build();
		RedCubeReturn = mecanumDrive.trajectorySequenceBuilder(RedCube3.end())
				.strafeRight(11.5)
				.forward(36.4)
				.addTemporalMarker(0.5, () -> SetWorm(neutralWormPos))
				.build();


		RedSphere4 = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
				.forward(1)
				.back(37.4)
				.strafeLeft(12)
				.addTemporalMarker(0.5, () -> SetWorm(leftWormPos))
				.addTemporalMarker(0.5, () -> SetLift(upperLevel))
				.addTemporalMarker(0.2, () -> SetClaw(false))
				.build();
		RedSphereReturn = mecanumDrive.trajectorySequenceBuilder(RedSphere4.end())
				.strafeRight(12)
				.forward(36.1)
				.addTemporalMarker(0.5, () -> SetWorm(neutralWormPos))
				.addTemporalMarker(0.5, () -> SetLift(groundLevel))
				.build();

		park = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
				.splineToLinearHeading(new Pose2d(-20, -30), Math.toRadians(-90))
				.build();
	}

	private void Run()
	{
		BluCubeSequence();
		sleep(1500);
		BluSphereSequence();
		sleep(1500);
		RedSphereSequence();
		sleep(1500);
		RedCubeSequence();
		sleep(1500);
		RedSphereSequence();
		sleep(1500);
		BluSphereSequence();
		sleep(1500);
		RedCubeSequence();
		sleep(500);
		Park();
	}

	private void BluCubeSequence()
	{
		mecanumDrive.followTrajectorySequence(BluCube1);
		sleep(100);
		SetClaw(true);
		sleep(50);
		mecanumDrive.followTrajectorySequence(BluCubeReturn);
	}

	private void BluSphereSequence()
	{
		mecanumDrive.followTrajectorySequence(BluSphere2);
		sleep(100);
		SetClaw(true);
		sleep(50);
		mecanumDrive.followTrajectorySequence(BluSphereReturn);
	}

	private void RedCubeSequence()
	{
		mecanumDrive.followTrajectorySequence(RedCube3);
		sleep(100);
		SetClaw(true);
		sleep(50);
		mecanumDrive.followTrajectorySequence(RedCubeReturn);
	}

	private void RedSphereSequence()
	{
		mecanumDrive.followTrajectorySequence(RedSphere4);
		sleep(100);
		SetClaw(true);
		sleep(50);
		mecanumDrive.followTrajectorySequence(RedSphereReturn);
	}

	private void Park()
	{
		mecanumDrive.followTrajectorySequence(park);
		SetClaw(false);
		sleep(400);
		SetClaw(true);
	}

	private void Telemetry()
	{
		telemetry.addData("Frame Count", webcam.getFrameCount());
		telemetry.addData("FPS", webcam.getFps());
		telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
		telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
		telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
		telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
//		telemetry.addData("Exposure: ", webcam.getExposureControl().getExposure(TimeUnit.MILLISECONDS));
//		telemetry.addData("Gain: ", webcam.getGainControl().getGain());
		telemetry.addLine();
		telemetry.addData("Object: ", pipeline.GetLastDetection().color.name() + " " + pipeline.GetLastDetection().shape.name());
		telemetry.addData("Worm pos: ", wormMotor.getCurrentPosition());
		telemetry.update();
	}

	private void SetClaw(boolean open)
	{
		catcher.setPosition(open ? 0f : 0.6f);
	}

	private void SetWorm(int pos)
	{
		wormMotor.setTargetPosition(pos);
		wormMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		wormMotor.setPower(1.0f);
	}

	private void SetLift(int pos)
	{
		liftMotor1.setTargetPosition(pos);
		liftMotor2.setTargetPosition(pos);
		liftMotor1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		liftMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
		liftMotor1.setPower(0.6f);
		liftMotor2.setPower(0.6f);
	}
}