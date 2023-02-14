package org.firstinspires.ftc.teamcode.autonom.Traiectorii;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.AutoPosition;
import org.firstinspires.ftc.teamcode.autonom.AutoUtil;
import org.firstinspires.ftc.teamcode.autonom.AutonomStangaHigh;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class TraiectoriiStangaHigh {
    AutonomStangaHigh auto;
    TrajectorySequence deliverPreload,catchCone1,deliverCone1, deliverCone2, catchCone2,catchCone3,catchCone4,catchCone5,deliverCone3,deliverCone4,deliverCone5,park1,park2,park3;

    public TraiectoriiStangaHigh(AutonomStangaHigh auto){
        this.auto = auto;
        initializeTrajectories();
    }
    double MAX_ANG_VEL = Math.toRadians(308.7320082135523);
    double TRACK_WIDTH = 13.2; // in

    private void initializeTrajectories(){
        deliverPreload = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .setConstraints
                        (SampleMecanumDrive.getVelocityConstraint(46,MAX_ANG_VEL,TRACK_WIDTH)
                                ,SampleMecanumDrive.getAccelerationConstraint(46))
                .strafeRight(53)
                .back(11.5)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-423))
                .build();
        catchCone1 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .forward(30.7)
                .addTemporalMarker(0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.35, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE5))
                .build();
        deliverCone1 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .back(30.7)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-423))
                .build();
        catchCone2 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .forward(31)
                .addTemporalMarker(0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.35, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE4))
                .build();
        deliverCone2 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .back(30.6)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-423))
                .build();
        catchCone3 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                ///.strafeLeft(0.1)
                .forward(30.5)
                .addTemporalMarker(0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.35, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE3))
                .build();
        deliverCone3 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .back(30.6)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-423))
                .build();
        catchCone4 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .forward(30.7)
                .addTemporalMarker( 0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.35, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.CONE2))
                .build();
        deliverCone4 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .back(30.3)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-423))
                .build();
        catchCone5 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .forward(30.4)
                .addTemporalMarker(0, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(.35, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
                .build();
        deliverCone5 = auto.mecanumDrive
                .trajectorySequenceBuilder(new Pose2d())
                .back(30.7)
                .addTemporalMarker(0, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.HIGH))
                .addTemporalMarker(.5, () -> AutoUtil.platePosition(auto.plateMotor,-423))
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
                .setConstraints
                        (SampleMecanumDrive.getVelocityConstraint(56,MAX_ANG_VEL,TRACK_WIDTH)
                                ,SampleMecanumDrive.getAccelerationConstraint(56))
                .forward(33.25)
                .addTemporalMarker(.0, () -> AutoUtil.platePosition(auto.plateMotor,-2423))
                .turn(Math.toRadians(90))
                .addTemporalMarker(1.8, () -> AutoUtil.platePosition(auto.plateMotor,-1423))
                .addTemporalMarker(2.4, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
               /// .addTemporalMarker(2, () -> AutoUtil.liftPosition(auto.liftMotor1, auto.liftMotor2, AutoPosition.ZERO))
                .build();
    }

    public void runAuto(int detected){
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.mecanumDrive.followTrajectorySequence(deliverPreload);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(250);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(150);
        auto.mecanumDrive.followTrajectorySequence(catchCone1);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(250);
        auto.mecanumDrive.followTrajectorySequence(deliverCone1);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(200);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(150);
        auto.mecanumDrive.followTrajectorySequence(catchCone2);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(250);
        auto.mecanumDrive.followTrajectorySequence(deliverCone2);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(200);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(150);
        auto.mecanumDrive.followTrajectorySequence(catchCone3);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(250);
        auto.mecanumDrive.followTrajectorySequence(deliverCone3);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(200);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(150);
        auto.mecanumDrive.followTrajectorySequence(catchCone4);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(100);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(250);
        auto.mecanumDrive.followTrajectorySequence(deliverCone4);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(200);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(150);
        auto.mecanumDrive.followTrajectorySequence(catchCone5);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(150);
        auto.AutoUtil.setClaw(auto.catcher,false);
        auto.sleep(200);
        auto.mecanumDrive.followTrajectorySequence(deliverCone5);
        auto.mecanumDrive.setPoseEstimate(new Pose2d(0, 0));
        auto.sleep(200);
        auto.AutoUtil.setClaw(auto.catcher,true);
        auto.sleep(100);
        if(detected == 2) auto.mecanumDrive.followTrajectorySequence(park2);
        else if(detected == 1) auto.mecanumDrive.followTrajectorySequence(park1);
        else auto.mecanumDrive.followTrajectorySequence(park3);
    }
}
