package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Config
@Autonomous(group = "autonom")
public class Autonom extends LinearOpMode {
    SampleMecanumDrive drive;
    SampleMecanumDrive mecanumDrive;
    Servo catcher;
    DcMotorEx liftMotor1,liftMotor2, plateMotor;
    AutoUtil AutoUtil = new AutoUtil();
    @Override
    public void runOpMode() throws InterruptedException {
        liftMotor1 = hardwareMap.get(DcMotorEx.class, "liftMotor1");
        liftMotor2 = hardwareMap.get(DcMotorEx.class, "liftMotor2");
        liftMotor1.setDirection(DcMotor.Direction.REVERSE);
        liftMotor2.setDirection(DcMotor.Direction.REVERSE);
        plateMotor = hardwareMap.get(DcMotorEx.class, "plateMotor");
        catcher = hardwareMap.get(Servo.class, "catcherServo");
        catcher.setPosition(.4f);


        initialize();
        waitForStart();
        while (opModeIsActive()) {

            new AutonomRed(this).runAuto();
            telemetry.addData("plateValue", plateMotor.getCurrentPosition());
            telemetry.addData("lift1Value", liftMotor1.getCurrentPosition());
            telemetry.addData("lift2Value", liftMotor2.getCurrentPosition());
            telemetry.update();
            sleep(30000);
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