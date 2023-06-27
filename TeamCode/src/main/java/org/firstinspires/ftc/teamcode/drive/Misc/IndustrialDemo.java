package org.firstinspires.ftc.teamcode.drive.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;


@Autonomous(group = "autonom")
public class IndustrialDemo extends LinearOpMode
{
	OpenCvWebcam webcam;
	CustomPipeline pipeline;
	int cameraMonitorViewId = 0;

	@Override
	public void runOpMode() throws InterruptedException
	{
		Init();

		waitForStart();

		pipeline.SetDebug(false);

		while (!isStopRequested())
			Run();
	}

	private void Init()
	{
		telemetry.setMsTransmissionInterval(100);

		pipeline = new CustomPipeline();

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
	}

	private void Run()
	{
		Telemetry();
	}

	private void Telemetry()
	{
		telemetry.addData("Frame Count", webcam.getFrameCount());
		telemetry.addData("FPS", webcam.getFps());
		telemetry.addData("Total frame time ms", webcam.getTotalFrameTimeMs());
		telemetry.addData("Pipeline time ms", webcam.getPipelineTimeMs());
		telemetry.addData("Overhead time ms", webcam.getOverheadTimeMs());
		telemetry.addData("Theoretical max FPS", webcam.getCurrentPipelineMaxFps());
		telemetry.addData("Exposure: ", webcam.getExposureControl().getExposure(TimeUnit.MILLISECONDS));
		telemetry.addData("Gain: ", webcam.getGainControl().getGain());
		telemetry.addLine();
		telemetry.addData("Object: ", pipeline.GetLastDetection().color.name() + " " + pipeline.GetLastDetection().shape.name());
		telemetry.update();
	}
}