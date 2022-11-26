package org.firstinspires.ftc.teamcode.autonom;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "autonom")
public class AutonomJunior extends LinearOpMode {

    SampleMecanumDrive drive;

    Servo claw;
    DcMotorEx liftMotor;

    Trajectory traj1;

    @Override
    public void runOpMode() throws InterruptedException {
        robotInit();

        waitForStart();
        while(!isStopRequested()){
            traj1 = drive.trajectoryBuilder(new Pose2d(0, 0)).forward(-50)
            .addDisplacementMarker(() -> {
                liftMotor.setTargetPosition(-2650);
                liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                liftMotor.setPower(1f);
            }).build();
            drive.followTrajectory(traj1);
            sleep(30000);
        }
    }

    private void robotInit(){
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(0, 0));
        claw = hardwareMap.get(Servo.class, "catcherServo");
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        claw.setPosition(.4f);
    }

}
