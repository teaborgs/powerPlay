package org.firstinspires.ftc.teamcode.drive.advanced;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoPosition;
import org.firstinspires.ftc.teamcode.autonom.AutoUtil;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable;

public class TraiectoriiStangaHighTest {
    AutonomStangaHighTest auto;
    TrajectorySequence deliverPreload,catchCone1,deliverCone,q, deliverCone2, catchCone2,catchCone3,catchCone4,catchCone5,deliverCone3,deliverCone4,deliverCone5,park1,park2,park3;
    public TraiectoriiStangaHighTest(AutonomStangaHighTest auto){
        this.auto = auto;
        initializeTrajectories();
    }
    Trajectory catchCone;
    double MAX_ANG_VEL = Math.toRadians(308.7320082135523);
    double TRACK_WIDTH = 13.2; // in
    private void initializeTrajectories(){
        deliverPreload = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .setConstraints
                        (SampleMecanumDrive.getVelocityConstraint(40,MAX_ANG_VEL,TRACK_WIDTH)
                                ,SampleMecanumDrive.getAccelerationConstraint(40))
                .strafeRight(54.05)
                .back(12.8)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-423))
                .build();
        deliverCone = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .back(30)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-423))
                .build();
        q = auto.mecanumDrive
                .trajectorySequenceBuilder(deliverPreload.end())
                .forward(15)
                .build();
        park3 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .back(9.9)
                .turn(Math.toRadians(90))
                .addTemporalMarker(.0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(1.4, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
                .build();
        park2 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .forward(13)
                .turn(Math.toRadians(90))
                .addTemporalMarker(.0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(1.7, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
                .build();
        park1 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .forward(33.25)
                .addTemporalMarker(.0, () -> AutoUtil.platePosition(auto.plateMotor,-2423))
                .turn(Math.toRadians(90))
                .addTemporalMarker(1.8, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(2.4, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
                /// .addTemporalMarker(2, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
                .build();
        catchCone = auto.mecanumDrive
                .trajectoryBuilder(deliverPreload.end())
                .forward(2)
                .build();
    }
    boolean colorPass()
    {
        if(auto.AutoUtil.colorDet(auto.sensor) == true)
            return true;
        return false;
    }
    ElapsedTime catchTime = new ElapsedTime();
    public void runAuto(int detected){
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.mecanumDrive.followTrajectorySequence(deliverPreload);
        auto.sleep(300);
        auto.AutoUtil.setClaw(auto.catcher, true);
        catchTime.reset();
        auto.mecanumDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        auto.mecanumDrive.followTrajectorySequence(q);
        while(!colorPass() && !auto.isStopRequested())
        {
            auto.mecanumDrive.setWeightedDrivePower(new Pose2d(.25,0));
            if (colorPass())
                auto.mecanumDrive.setWeightedDrivePower(new Pose2d(0,0));
        }
        auto.mecanumDrive.setWeightedDrivePower(new Pose2d(0,0));
    }
}
