package org.firstinspires.ftc.teamcode.drive.Misc;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.autonom.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


@Autonomous(group = "autonom")
public class IndustrialDemo extends LinearOpMode
{
    SampleMecanumDrive mecanumDrive;
    OpenCvCamera camera;
    CustomPipeline pipeline;
    int cameraMonitorViewId = 0;

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
        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new CustomPipeline();

        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });


        telemetry.setMsTransmissionInterval(100);
    }

    private void Run()
    {
        Telemetry();
    }

    private void Telemetry()
    {
        telemetry.addLine(pipeline.GetLastDetection().name());
        telemetry.update();
    }
}