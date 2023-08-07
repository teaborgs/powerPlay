package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonom.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.autonom.Traiectorii.TraiectoriiStangaMid;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import java.util.ArrayList;


@Config
@Autonomous(group = "autonom")
public final class AutonomStangaMid extends LinearOpMode
{
	public SampleMecanumDrive mecanumDrive;
	public Servo catcher;
	public DcMotorEx liftMotor1, liftMotor2, wormMotor;

	OpenCvCamera camera;
	AprilTagDetectionPipeline aprilTagDetectionPipeline;


	int detected = 3; // Default zone is 3

	@Override
	public void runOpMode() throws InterruptedException
	{
		Init();

		while (!isStarted() && !isStopRequested())
			Detectie();

		while (opModeIsActive() && !isStopRequested()) // TODO: testing
		{
			new TraiectoriiStangaMid(this).RunAuto(detected);
			sleep(30000);
		}
	}

	private void Detectie()
	{
		AprilTagDetection tagOfInterest = ReadPipeline();

		if(tagOfInterest != null)
			detected = tagOfInterest.id;

		telemetry.addData("obiect", detected);
		telemetry.update();
	}

	private AprilTagDetection ReadPipeline()
	{
		ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

		if(currentDetections.size() != 0)
		{
			for (AprilTagDetection tag : currentDetections)
				if (tag.id == 1 || tag.id == 2 || tag.id == 3)
					return tag;
		}
		return null;
	}

	private void Init()
	{
		// Init Hardware
		mecanumDrive = new SampleMecanumDrive(hardwareMap);
		mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
		mecanumDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		mecanumDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

		liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
		liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
		wormMotor = hardwareMap.get(DcMotorEx.class, "wormMotor");
		catcher = hardwareMap.get(Servo.class, "catcherServo");
		liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
		wormMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

		liftMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		liftMotor1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		liftMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		liftMotor2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
		wormMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
		wormMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

		// Init Camera
		int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()); // TODO: testing
		camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

		// Init Detection Pipeline
		double fx = 578.272;
		double fy = 578.272;
		double cx = 402.145;
		double cy = 221.506;
		double tagSize = 0.166; // meters
		aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);

		// Enable Camera
		camera.setPipeline(aprilTagDetectionPipeline);
		camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
		{
			@Override
			public void onOpened() { camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT); }

			@Override
			public void onError(int errorCode) { telemetry.addData("Cam error", errorCode); }
		});

		AutoUtil.setClaw(catcher, false);

		telemetry.setMsTransmissionInterval(50);
	}
}