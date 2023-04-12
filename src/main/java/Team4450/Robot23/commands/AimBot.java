package Team4450.Robot23.commands;

import Team4450.Lib.*;

import org.opencv.core.Rect2d;
import org.photonvision.PhotonCamera;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import Team4450.Robot23.subsystems.Arm;
import Team4450.Robot23.subsystems.Claw;
import Team4450.Robot23.subsystems.Winch;
import Team4450.Robot23.subsystems.DriveBase;
import Team4450.Robot23.subsystems.LimeLight;
import Team4450.Robot23.subsystems.Winch;
import Team4450.Robot23.Constants;
import Team4450.Robot23.Constants.*;

import org.opencv.core.Rect;


public class AimBot extends CommandBase {

    //kEndGoalY is temporary
    private static double           kP = .3, kI = 0.03, kD = 0.0, kEndGoalY = 0.0, kP2 = .3, kI2 = 0.03, kD2 = 0.0, kToleranceX = 0.0, kToleranceY = 0.0;
    private double                  kToleranceDeg = .5;
   
    private double                  startTime, tempTime, elapsedTime, latestTimestamp;
    private boolean                 noTargets = false, hadTargets;

    private Arm                     arm;
    private Winch                   winch;
    private Claw                    claw;
    private DriveBase               driveBase;
    private PhotonCamera            phCamera;
    private AprilTagFieldLayout     tagLayout;
    
    private Pose3d                  latestTargetPose, targetPolePose;
    private Pose2d                  latestAprilPose2d, latestRobotPose, targetPose2d;
    private Translation2d           translationTarget;
    private Translation3d           limeLightToCenter;
    private Transform2d             transformTarget;

    private SynchronousPID          pidS = new SynchronousPID(kP, kI, kD);
    private SynchronousPID          pidT = new SynchronousPID(kP2, kI2, kD2);

    private LimeLight               limeLight = new LimeLight();

    private Rect                    targetTape;
    
    private ScoreMode               scoreMode;

    // private SequentialCommandGroup	commands = null;
	// private Command					command = null;
    private PhotonTrackedTarget     target;  
    


    public AimBot(PhotonCamera phCamera,
                            DriveBase driveBase,
                            AprilTagFieldLayout tagLayout,
                            Translation3d limeLightToCenter,
                            ScoreMode scoreMode)
    {
       Util.consoleLog();

        this.phCamera = phCamera;
        this.driveBase = driveBase;
        this.tagLayout = tagLayout;
        this.limeLightToCenter = limeLightToCenter;
        this.scoreMode = scoreMode;

    }

    @Override
    public void initialize(){
        
        Util.consoleLog();

        pidS.setOutputRange(0, 0.5);
        pidT.setOutputRange(0, 0.5);

        phCamera.setPipelineIndex(0);

        startTime = Util.timeStamp();
        tempTime = Util.timeStamp();
        
    }

    @Override
    public void execute(){
        
        var result = phCamera.getLatestResult();

        if(scoreMode == ScoreMode.CUBE && result.hasTargets()){

            hadTargets = true;

            elapsedTime = Util.getElaspedTime(tempTime);

            target = result.getBestTarget();

            latestAprilPose2d = (Constants.APRILTAGFIELDLAYOUT.getTags().get(target.getFiducialId()).pose).plus(target.getBestCameraToTarget().inverse()).toPose2d();
            latestTimestamp = Util.getElaspedTime();
            
            if(target.getFiducialId() < 4)
                translationTarget = new Translation2d(-0.1016, -0.0762);
            else    
                translationTarget = new Translation2d(0.1016, 0.0762);

            transformTarget = new Transform2d(translationTarget, new Rotation2d(0.0));

            targetPose2d = (tagLayout.getTags().get(target.getFiducialId()).pose.toPose2d().transformBy(new Transform2d(translationTarget, new Rotation2d(0.0)))).plus(transformTarget);

            pidS.setSetpoint(targetPose2d.getX());
            pidT.setSetpoint(targetPose2d.getY());

            //merges PhotonVision pose and Odometry pose to calculate to the lateset robot pose
            driveBase.getOdometry().addVisionMeasurement(latestAprilPose2d, latestTimestamp);
            latestRobotPose = driveBase.getOdometry().getEstimatedPosition();
            
            //drive in the drection, rotation is temp
            driveBase.drive(-pidS.calculate(latestRobotPose.getX(), elapsedTime), -pidT.calculate(latestRobotPose.getY(), elapsedTime),
                            0.0);


            Util.consoleLog("PhotonHasTargets:%b, LimeHasTargets:%b, HadTargets:%b, ScoreMode:%s, /n PoseX=%d, PoseY =%d", 
                            result.hasTargets(), limeLight.targetVisible(), hadTargets, scoreMode.name(), 
                            targetPose2d.getX(), targetPose2d.getY());
        
        }
        
        else if(scoreMode == ScoreMode.CONE) {
            
            phCamera.setPipelineIndex(1);

            if(limeLight.targetVisible()){
                //now has had targets
                hadTargets = true;

                elapsedTime = Util.getElaspedTime(tempTime);

                //gets the retangle of the tape target
                targetTape = limeLight.getTargetRectangle();

                pidS.setSetpoint(0.0);
                pidT.setSetpoint(0.0);

                //creates a time for both pid2s
                elapsedTime = Util.getElaspedTime(tempTime);

                //Enters modified power values based on offset from tape target
                driveBase.drive(-pidS.calculate(-limeLight.offsetX(), elapsedTime), 
                                -pidT.calculate(-limeLight.offsetY() - kEndGoalY, elapsedTime), 0.0);
                
                Util.consoleLog("PhotonHasTargets:%b, LimeHasTargets:%b, HadTargets:%b, ScoreMode:%s", 
                result.hasTargets(), limeLight.targetVisible(), hadTargets, scoreMode.name());
            }
        }

        else if(hadTargets && (target.getFiducialId() != 0 || targetTape != null)){
                
            latestRobotPose = driveBase.getOdometry().getEstimatedPosition();

            elapsedTime = Util.getElaspedTime(tempTime);

            //claculate distance between aprilTag and robot (turn pose2d ---> pose3d)
            //targetToRobotDist = Math.sqrt(Math.pow(Math.abs(targetPose2d.getX() - latestRobotPose.getX()), 2.0)
                //+ Math.pow(Math.abs(targetPose2d.getY() - latestRobotPose.getY()), 2.0));

            //get the direction the robot needs to go in
            //throttleTime = (targetToRobotDist * Math.sin(result.getBestTarget().getYaw())/1.0);
            //strafeTime = (targetToRobotDist * Math.cos(result.getBestTarget().getYaw())/1.0);
            
            //drive in the drection
            driveBase.drive(-pidS.calculate(latestRobotPose.getX(), elapsedTime), -pidT.calculate(latestRobotPose.getY(), elapsedTime),
                            0.0);

        }
        else {
            //add error message
            Util.consoleLog("No visable target & no previous targets.");
            Util.consoleLog("PhotonHasTargets:%b, LimeHasTargets:%b, HadTargets:%b, ScoreMode:%s", 
                            result.hasTargets(), limeLight.targetVisible(), hadTargets, scoreMode.name());
            noTargets = true;
        }
    }

    @Override
    public boolean isFinished(){

        return (pidS.onTarget(kToleranceX) && pidT.onTarget(kToleranceY)) || noTargets;

    }

    @Override
    public void end(boolean interrupted){
        //score with arm
        /* 
        commands = new SequentialCommandGroup();
        commands.addCommands(new ArmWinchPresets(arm, winch, armTargetPose));
        commands.schedule();
        claw.setClawState(ClawPosition.OPEN);
        */

        //turns off limelight
        phCamera.setLED(VisionLEDMode.kOff);

        if(hadTargets)
            driveBase.drive(translationTarget.getX(), translationTarget.getY(), 0.0);

        //resets hadTargets
        hadTargets = false;

        Util.consoleLog("interrupted=%b", interrupted);
    }
}
