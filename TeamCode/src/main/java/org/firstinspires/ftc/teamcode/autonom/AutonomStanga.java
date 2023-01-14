package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.AutoPosition;
import org.firstinspires.ftc.teamcode.autonom.OpenCV.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.autonom.Traiectorii.TraiectoriiStanga;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
@Autonomous(group = "autonom")
public class AutonomStanga extends LinearOpMode {

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;
    int cameraMonitorViewId = 0;
    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    int Left = 1;
    int Middle = 2;
    int Right = 3;

    public AprilTagDetection tagOfInterest = null;

    public SampleMecanumDrive mecanumDrive;
    public Servo catcher;
    public DcMotorEx liftMotor1, liftMotor2, plateMotor;
    public AutoUtil AutoUtil = new AutoUtil();
    int detected = 3;
    @Override
    public void runOpMode() throws InterruptedException {
        liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
        plateMotor = hardwareMap.get(DcMotorEx.class, "plateMotor");
        catcher = hardwareMap.get(Servo.class, "catcherServo");
        catcher.setPosition(.4f);
        liftMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        plateMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
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

        telemetry.setMsTransmissionInterval(50);

        initialize();
        while (!isStarted() && !isStopRequested()) {
            detectie();
            if(tagOfInterest != null)
                detected = tagOfInterest.id;
            telemetry.addData("obiect", detected);
            telemetry.update();
        }
        while (opModeIsActive() && !isStopRequested()) {
            new TraiectoriiStanga(this).runAuto(detected);
            sleep(30000);
        }
    }
    void detectie() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if(currentDetections.size() != 0) {
            boolean tagFound = false;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == Left || tag.id == Middle || tag.id == Right) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }
        }
    }
    private void initialize(){
        liftMotor1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        plateMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        AutoUtil.setClaw(catcher,false);
    }
}