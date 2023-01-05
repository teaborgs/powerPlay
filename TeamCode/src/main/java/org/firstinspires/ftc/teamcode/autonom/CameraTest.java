package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.ClassFactoryImpl;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "autonom")
public class CameraTest extends LinearOpMode {
    SampleMecanumDrive mecanumDrive;
    String vuforiaKey = "Aaiq/1//////AAABmR5nE1/0E0LclZpr6AaY5+A2o36In7uJDJ6OQngVynh2aDFKeiUTZQggihn/8KkhWmh5Jnb9cj7GU4nRu0leL6fxUJ4jg2j/4x2W+eVBwqiHHJPwMfYElGUwFiCT9CycVyk+lycCrUcMQrUMe2Aq0kWxMD3xbMDWBVUq2V3ceG6ec9GGYF/HRjVx2FoGFsiuxziwYFY/mKGN8l2kMvYvYdCog0XgHWMi5lfHo/cg0kXeVBYx72I7xD6pXuGMZlf3Lhk61R0iKn0uJ+rnZdc9UWpFhyQTokQDTCiJ5wm3eNShGn5qLSeIyw2w0wLWtLRRBlJEgxc2LOQeDjogMAIiXSvIw6pAbkRR8QflyUpNQ4j5";

    VuforiaLocalizer vuforia;

    ClassFactory classFactory = new ClassFactoryImpl(); // idk
    @Override
    public void runOpMode() throws InterruptedException {
        initialize();
        while (opModeIsActive()) {

            telemetry.update();
            sleep(30000);
        }
    }
    private void initialize(){
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        params.vuforiaLicenseKey = vuforiaKey;
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
        params.cameraDirection = VuforiaLocalizer.CameraDirection.BACK; // dubios
        vuforia = classFactory.createVuforia(params);
        waitForStart();
    }
}