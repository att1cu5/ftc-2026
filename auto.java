package org.firstinspires.ftc.teamcode;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
//import static android.os.SystemClock.sleep;
//import static org.firstinspires.ftc.robotcontroller.external.samples.ConceptAprilTagEasy.USE_WEBCAM;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.linearOpMode;
//import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
//import static org.firstinspires.ftc.vision.VisionPortal.CameraState.STREAMING;

//import androidx.core.util.SparseBooleanArrayKt;

import org.firstinspires.ftc.vision.VisionPortal;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Collections;
import java.util.List;

@Autonomous (name = "auto (Blocks to Java)", group="LinearOpMode") //this is fine
public class auto extends LinearOpMode { //this is fine

    double Abcd=0;
    double fixedtheta=65;//find this
    double rotater=0;
    double test=0;



    double motifs=0;
    double speedOfintakeOff=0;
    double speedOfintakeOn=0.8;
    double x=0;
    double y=0;
    double turn=0;
    double backleft_A;
    double backright_A;
    double frontleft_A;
    double frontright_A;
    double deltaX=0;
    double startx=0;
    double CurX=0;
    double CurY=0;
    double runtime=0;
    double starty=0;
    double deltaY=0;
    boolean USE_WEBCAM;
    AprilTagProcessor myAprilTagProcessor;
    Position cameraPosition;
    YawPitchRollAngles cameraOrientation;
    VisionPortal myVisionPortal;
    double rangeB=0;

    double pointAx=-16.5354;//tune this in inches times -1
    double pointBx=0.0001;
    double pointCx=0;
    double pointAy=0.0001;
    double pointBy=16.5354;// robot height of camera in inches



    double currentpositionY=0;
    double currentpositionX=0;

    boolean thisExpUp;
    boolean thisExpDn;
    boolean thisGainUp;
    boolean thisGainDn;
    boolean lastExpUp;
    boolean lastExpDn;
    boolean lastGainUp;
    boolean lastGainDn;
    double onerev=383.6;
    double desiredspeed=0;
    double intialspeed=0; //intial speed before change measured in ticks adjust value


    public double change=10; //adjust value
    public double degree1=0.5;
    public double degree2=0;
    public double latchopen=0.5;
    public double latchclose=0;
    public double artifactholderopen=0.5; // adjust value in the future
    public double artifactholderclose=0; // adjust value in the future
    public double shooterholderopen=0.5; // adjust value in the future
    public double shooterholderclose=0; // adjust value in the future
    public double beltspeed1=-1; // adjust value in the future
    public double beltspeed2=1; // adjust value in the future
    //in mm

    double[] motif={0,0,0,0};
    double[] correctmotif={0,0,0,0};
    double red=0;
    double green=0;
    double blue=0;

    double sense=0;
    double Xa=0;
    double Ya=0;

    double Za=0;
    double Pitch=0;
    double Yaw=0;
    double myYAW=0;
    double Roll=0;
    double Ycenter=0;
    double Xcenter=0;
    int i=0;
    double A=0;
    double B=0;
    double C=0;

    @Override
    public void runOpMode() throws InterruptedException {
    //private ElapsedTime runtime = new ElapsedTime();
    USE_WEBCAM = true;
    DcMotor backleft;
    IMU imu;
    DcMotor backright;
    DcMotor frontright;
    DcMotor frontleft;
    Servo pivotintake;
    Servo pivotintakeA;
    Servo shooterholder;
    Servo artifactholder;
    CRServo belt;
    CRServo holder;
    DcMotor X;
    final ElapsedTime runtime = new ElapsedTime();
    DcMotor intake;
    DcMotor shooterwheelA;
    DcMotor shooterwheelB;

    boolean USE_WEBCAM = true;


    shooterholder = hardwareMap.get(Servo.class, "shooterholder");
    artifactholder = hardwareMap.get(Servo.class, "artifactholder");
    shooterwheelA = hardwareMap.get(DcMotor.class, "shooterwheelA");
    shooterwheelB = hardwareMap.get(DcMotor.class, "shooterwheelB");
    holder = hardwareMap.get(CRServo.class, "holder");
    X=hardwareMap.get(DcMotor.class, "X");
    intake = hardwareMap.get(DcMotor.class, "intake");
    backleft = hardwareMap.get(DcMotor.class, "backleft");
    backright = hardwareMap.get(DcMotor.class, "backright");
    frontright = hardwareMap.get(DcMotor.class, "frontright");
    frontleft = hardwareMap.get(DcMotor.class, "frontleft");
    intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    X.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    shooterwheelA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    shooterwheelB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    shooterwheelB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    shooterwheelA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    X.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
    //startup hi=new startup();
    //hi.initAprilTag();



    double CurY=0;
    double CurX=0;
    double filteredG=0;
    double filteredF=0;
    double filtered=0;
    double filteredA=0;
    double filteredC=0;
    double filteredB=0;
    double filteredV=0;
    double filteredD=0;
    runtime.reset();
    waitForStart();
     
    while (opModeIsActive()) {



        
        double posY=shooterwheelA.getCurrentPosition();
        filter lpd=new filter();
        filteredG=lpd.filterinput(0.3,posY,filteredG);
        CurY=filteredG;
        double posX=X.getCurrentPosition();
        filter lpa=new filter();
        filteredF=lpa.filterinput(0.3,posX,filteredF);
        CurX=filteredF;
      //  List<AprilTagDetection> myAprilTagDetections = Collections.emptyList();
        //AprilTagDetection myAprilTagDetection = null;

        // Get a list of AprilTag detections.



        //telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));

        //for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
          //  myAprilTagDetection = myAprilTagDetection_item;
            // Display info about the detection.


//            if (myAprilTagDetection.metadata != null) {

                // Only use tags that don't have Obelisk in them since Obelisk tags don't have valid location data
  //              double test=myAprilTagDetection.id;
    //            double motifs=test;
      //          startup logic=new startup();
        //        if (!logic.contains(myAprilTagDetection.metadata.name, "Obelisk")) {
        //            double currentpositionY=shooterwheelA.getCurrentPosition();
         //           double currentpositionX=X.getCurrentPosition();
           //         double Ya=0;
            //        double Xa=0;
            //        double Za=0;
            //        double Pitch=0;
            //        double Roll=0;
            //        double Yaw=0;
            //        double rangeB=0;
            //        double pointCx=0;
            //        double offsetX=9.129397076417323;
            //        Ya=Math.round(myAprilTagDetection.robotPose.getPosition().y*10);
            //        Xa=Math.round(myAprilTagDetection.robotPose.getPosition().x*10);
            //        Za=Math.round(myAprilTagDetection.robotPose.getPosition().z*10);
            //        Pitch=Math.round(myAprilTagDetection.robotPose.getOrientation().getPitch()*10);
            //        Roll=Math.round(myAprilTagDetection.robotPose.getOrientation().getRoll()*10);
            //        Yaw=Math.round(myAprilTagDetection.robotPose.getOrientation().getYaw()*10);
            //        double startx=Xa;
            //        double starty=Ya;
            //        rangeB=Math.sqrt(Math.pow(((startx - currentpositionX) * Math.PI * 1.25984) / 2000, 2) + Math.pow(((starty - currentpositionY) * Math.PI * 1.25984) / 2000, 2));
            //        pointCx=rangeB+offsetX;
            //        double[] correctmotif={0,0,0,0};
            //        telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().y, 6, 1)+ " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().z, 6, 1) + "  (inch)");
            //        if(motifs!=20 && motifs!=24){
            //            if(motifs==23){
            //                correctmotif[0]=2;
            //                correctmotif[1]=2;
            //                correctmotif[2]=1;
            //            }
            //            if(motifs==21){
            //                correctmotif[0]=1;
            //                correctmotif[1]=2;
                      //      correctmotif[2]=2;
                     //   }
                    //    if(motifs==22){
                      //      correctmotif[0]=2;
                     //       correctmotif[1]=1;
                    //        correctmotif[2]=2;
                  //      }





                //    }

              //  }
            //} else {

          //  }

        //}


        double Circumference=76.8*Math.PI;
        double intialspeedA=0.1;//CHANGE LATER
        FeedforwardA termA= new FeedforwardA();
        PIDCONTOLLERshooterA ShooterA=new PIDCONTOLLERshooterA();
        double posA=frontright.getCurrentPosition();
        filter lp=new filter();
        filtered=lp.filterinput(0.3,posA,filtered);
        double currentspeedA=((filtered/383.6)*104*2*Math.PI*(96/32))/runtime.seconds();
        double VelocityA=0.1;//tune this
        double AccelerationA=0.1;//tune this
        double KSshooterA=0.1;//tune this
        double KVshooterA=0.1;//tune this
        double KAshooterA=0.1;//tune this\
        double feedforwardA=0;
        feedforwardA=termA.feedforwardtermA(VelocityA, AccelerationA, KSshooterA, KVshooterA, KAshooterA);
        double desiredspeedA=0.1;//CHANGE LATER
        double speedAA=0;
        speedAA=ShooterA.calcshooterA(desiredspeedA,currentspeedA);
        double speedA=speedAA;
        speedA=speedA+feedforwardA;
        double intialspeedC=0.1;//CHANGE LATER
        FeedforwardC termC=new FeedforwardC();
        PIDCONTOLLERshooterC ShooterC=new PIDCONTOLLERshooterC();
        double posC=frontleft.getCurrentPosition();
        filter lpf=new filter();
        filteredA=lpf.filterinput(0.3,posC,filteredA);
        double currentspeedC=((filteredA/383.6)*104*2*Math.PI*(96/32))/runtime.seconds();
        double VelocityC=0.1;//tune this
        double AccelerationC=0.1;//tune this
        double KSshooterC=0.1;//tune this
        double KVshooterC=0.1;//tune this
        double KAshooterC=0.1;//tune this
        double feedforwardC=termC.feedforwardtermC(VelocityC, AccelerationC, KSshooterC, KVshooterC, KAshooterC);
        double desiredspeedC=0.1;//CHANGE LATER
        double speedCC=0;
        speedCC=ShooterC.calcshooterC(desiredspeedC,currentspeedC);
        double speedC=speedCC;
        speedC=speedC+feedforwardC;
        double intialspeedD=0.1;//CHANGE LATER
        FeedforwardD termD=new FeedforwardD();
        PIDCONTOLLERshooterD ShooterD=new PIDCONTOLLERshooterD();
        double posD=backright.getCurrentPosition();
        filter lpg=new filter();
        filteredB=lpg.filterinput(0.3,posD,filteredB);
        double currentspeedD=((filteredB/383.6)*104*2*Math.PI*(96/32))/runtime.seconds();
        double VelocityD=0.1;//tune this
        double AccelerationD=0.1;//tune this
        double KSshooterD=0.1;//tune this
        double KVshooterD=0.1;//tune this
        double KAshooterD=0.1;//tune this
        double feedforwardD=termD.feedforwardtermD(VelocityD, AccelerationD, KSshooterD, KVshooterD, KAshooterD);
        double desiredspeedD=0.1;//CHANGE LATER
        double speedDD=0;
        speedDD=ShooterD.calcshooterD(desiredspeedD,currentspeedD);
        double speedD=speedDD;
        speedD=speedD+feedforwardD;
        double intialspeedE=0.1;//CHANGE LATER
        FeedforwardE termE=new FeedforwardE();
        PIDCONTOLLERshooterE ShooterE=new PIDCONTOLLERshooterE();
        double posR=backleft.getCurrentPosition();
        filter lpp=new filter();
        filteredC=lpp.filterinput(0.3,posR,filteredC);
        double currentspeedE=((filteredC/383.6)*104*2*Math.PI*(96/32))/runtime.seconds();
        double VelocityE=0.1;//tune this
        double AccelerationE=0.1;//tune this
        double KSshooterE=0.1;//tune this
        double KVshooterE=0.1;//tune this
        double KAshooterE=0.1;//tune this
        double feedforwardE=termE.feedforwardtermE(VelocityE, AccelerationE, KSshooterE, KVshooterE, KAshooterE);
        double desiredspeedE=0.1;//CHANGE LATER
        double speedEE=0;
        speedEE=ShooterE.calcshooterE(desiredspeedE,currentspeedE);
        double speedE=speedEE;
        speedE=speedE+feedforwardE;
        double desX=-10000;//tune later
        double desY=-2; //tune later
        telemetry.addData("posX",CurX);
        telemetry.update();
        while(CurX>desX){
            double posO=X.getCurrentPosition();
            filter lppd=new filter();
            filteredV=lppd.filterinput(0.3,posO,filteredV);
            CurX=filteredV;
            telemetry.addData("posX",CurX);
            frontright.setPower(speedA);
            frontleft.setPower(speedC);
            backleft.setPower(-speedE);
            backright.setPower(-speedD);
            telemetry.update();
            if(CurX==desX || CurX<desX){
                frontright.setPower(0);
                frontleft.setPower(0);
                backleft.setPower(0);
                backright.setPower(0);
                break;
            }
        }

        //while(CurY>desY){
          //  CurY=shooterwheelA.getCurrentPosition();
        //    telemetry.addData("posY",CurY);
            
         //   frontright.setPower(speedA);
          //  frontleft.setPower((speedC+0.2));
    //        backleft.setPower((speedE+0.2));
     //       backright.setPower(speedD);
       //     telemetry.update();
         //   if(CurY==desY || CurY<desY){
           //     frontright.setPower(0);
        //        frontleft.setPower(0);
        //        backleft.setPower(0);
        //        backright.setPower(0);
        //        break;

        //    }
        //}

        double offsetY=5.1496063;
        double pointCy=38.759843+offsetY;
        double fixedtheta=0;//tune
        double gravity=386.08858267717;
        double intialspeed=calcPhys.answervelocity(pointCy, gravity, fixedtheta);
        FeedforwardB termB=new FeedforwardB();
        PIDCONTOLLERshooterB ShooterB=new PIDCONTOLLERshooterB();
        double posE=shooterwheelB.getCurrentPosition();
        filter lps=new filter();
        filteredD=lps.filterinput(0.3,posE,filteredD);
        double currentspeedB=((filteredD/383.6)*Circumference*(96/32))/runtime.seconds();
        double VelocityB=0;//tune this
        double AccelerationB=0;//tune this
        double KSshooterB=0;//tune this
        double KVshooterB=0;//tune this
        double KAshooterB=0;//tune this
        double feedforwardB=termB.feedforwardtermB(VelocityB, AccelerationB, KSshooterB, KVshooterB, KAshooterB);

        double desiredspeed=0;//CHANGE LATER
        double speedBB=0;
        speedBB=ShooterB.calcshooterB(desiredspeed,currentspeedB);
        double speedB=0;
        speedB=speedBB+feedforwardB;
        //shooterwheelB.setPower(speedB);
        //shooterwheelA.setPower(speedB);
        FeedforwardAang termAang=new FeedforwardAang();
        PIDcontrollerAang shooterAang=new PIDcontrollerAang();
        double currentspeedAang=((frontright.getCurrentPosition()/383.6)*(96/32)*104*2*Math.PI)/runtime.seconds();
        double VelocityAang=0;//tune this
        double AccelerationAang=0;//tune this
        double KSshooterAang=0;//tune this
        double KVshooterAang=0;//tune this
        double KAshooterAang=0;//tune this
        double feedforwardAang=termAang.feedforwardtermAang(VelocityAang, AccelerationAang, KSshooterAang, KVshooterAang, KAshooterAang);

        double desiredspeedAang=0;//CHANGE LATER

        double speedAangAang=shooterAang.calcAang(desiredspeedAang);
        speedAangAang=feedforwardAang+speedAangAang;
        FeedforwardDang termDang=new FeedforwardDang();
        PIDcontrollerDang shooterDang=new PIDcontrollerDang();
        double currentspeedDang=((backright.getCurrentPosition()/383.6)*(96/32)*104*2*Math.PI)/runtime.seconds();
        double VelocityDang=0;//tune this
        double AccelerationDang=0;//tune this
        double KSshooterDang=0;//tune this
        double KVshooterDang=0;//tune this
        double KAshooterDang=0;//tune this
        double feedforwardDang=termDang.feedforwardtermDang(VelocityDang, AccelerationDang, KSshooterDang, KVshooterDang, KAshooterDang);

        double desiredspeedDang=0;//CHANGE LATER

        double speedDangDang=shooterDang.calcDang(desiredspeedDang);
        speedDangDang=speedDangDang+feedforwardDang;
        FeedforwardEang termEang=new FeedforwardEang();

        PIDcontrollerEang shooterEang=new PIDcontrollerEang();
        double currentspeedEang=((backleft.getCurrentPosition()/383.6)*(96/32)*104*2*Math.PI)/runtime.seconds();
        double VelocityEang=0;//tune this
        double AccelerationEang=0;//tune this
        double KSshooterEang=0;//tune this
        double KVshooterEang=0;//tune this
        double KAshooterEang=0;//tune this
        double feedforwardEang=termEang.feedforwardtermEang(VelocityEang, AccelerationEang, KSshooterEang, KVshooterEang, KAshooterEang);

        double desiredspeedEang=0;//CHANGE LATER

        double speedEangEang=shooterEang.calcEang(desiredspeedEang);
        FeedforwardCang termCang=new FeedforwardCang();

        PIDcontrollerFang shooterFang=new PIDcontrollerFang();
        double currentspeedFang=((frontleft.getCurrentPosition()/383.6)*(96/32)*104*2*Math.PI)/runtime.seconds();
        double VelocityFang=0;//tune this
        double AccelerationFang=0;//tune this
        double KSshooterFang=0;//tune this
        double KVshooterFang=0;//tune this
        double KAshooterFang=0;//tune this
        double feedforwardFang=termCang.feedforwardtermCang(VelocityFang, AccelerationFang, KSshooterFang, KVshooterFang, KAshooterFang);

        double desiredspeedCang=0;//CHANGE LATER

        double speedCangCang=shooterFang.calcFang(desiredspeedCang);
        speedCangCang=feedforwardFang+speedCangCang;
        //frontright.setPower(speedAangAang);
        //frontleft.setPower(speedCangCang);
        //backleft.setPower(speedEangEang);
        //backright.setPower(speedDangDang);
        motorintake state=new motorintake();
        double speedOfIntake=0;//set this
        if (speedOfIntake>speedOfintakeOn){
            speedOfIntake=speedOfintakeOn;
        }
        if (speedOfIntake<-speedOfintakeOn){
            speedOfIntake=-speedOfintakeOn;
        }

        double stateofintake=state.set_intake(speedOfIntake);
        //intake.setPower(stateofintake);
        double angleofmotorA=0;//set this
        Servos_A degrees=new Servos_A();
        if(Servos_A.angles(angleofmotorA)<=180){
           double timesx=(Servos_A.angles(angleofmotorA)/180);
           angleofmotorA=timesx;
           shooterholder.setPosition(angleofmotorA);
        }
        double angleofmotorB=0;//set this
        Servos_A degreesB=new Servos_A();
        if(Servos_A.angles(angleofmotorB)<=180){
            double timesxs=(Servos_A.angles(angleofmotorB)/180);
            angleofmotorB=timesxs;
            artifactholder.setPosition(angleofmotorB);
        }
        double holderpower=0;//set this
        Servos_B powerofintakes=new Servos_B();
        if(powerofintakes.direction(holderpower)>1){
            holderpower=1;
        }
        if(powerofintakes.direction(holderpower)<-1){
            holderpower=-1;
        }
        //holder.setPower(holderpower);

        sleep(20);








    }
}

    public class nothing {
        public double nothing(double C) {
            return C;
        }
    }

    public class PIDcontrollerCang {
        public double kpCang = 0;
        public double kiCang = 0;
        public double kdCang = 0;
        public double setPointCang = 0;
        double lasterrorCAng = 0;
        double outputCAng = 0;
        double IntergralSumCang = 0;

        public double PIDcontrollerCang(double kpCang, double kiCang, double kdCang, double setPointCang) {
            this.kpCang = kpCang;
            this.kiCang = kiCang;
            this.kdCang = kdCang;
            this.setPointCang = setPointCang;
            return 0;
        }
        public double calcCang(double MesurementCang){
            double errorCang=setPointCang-MesurementCang;
            double kpCangTerm=kpCang*errorCang;
            IntergralSumCang+=errorCang;
            double kiCangTerm=kiCang*IntergralSumCang;
            double KdCangTerm=(errorCang-lasterrorCAng);
            double KdCangTermA=kdCang*KdCangTerm;
            double outputCAng=kiCangTerm+KdCangTermA+kpCangTerm;
            lasterrorCAng=errorCang;
            return outputCAng;
        }
    }


}
class PIDcontrollerFang {
    public double kpFang = 0;
    public double kiFang = 0;
    public double kdFang = 0;
    public double setPointFang = 0;
    double lasterrorFAng = 0;
    double outputFAng = 0;
    double IntergralSumFang = 0;

    public double PIDcontrollerFang(double kpFang, double kiFang, double kdFang, double setPointFang) {
        this.kpFang = kpFang;
        this.kiFang = kiFang;
        this.kdFang = kdFang;
        this.setPointFang = setPointFang;
        return 0;
    }
    public double calcFang(double MesurementFang){
        double errorFang=setPointFang-MesurementFang;
        double kpFangTerm=kpFang*errorFang;
        IntergralSumFang+=errorFang;
        double kiFangTerm=kiFang*IntergralSumFang;
        double KdFangTerm=(errorFang-lasterrorFAng);
        double KdFangTermA=kdFang*KdFangTerm;
        double outputFAng=kiFangTerm+KdFangTermA+kpFangTerm;
        lasterrorFAng=errorFang;
        return outputFAng;
    }
}




class PIDcontrollerDang{
    public double kpDang=0;
    public double kiDang=0;
    public double kdDang=0;
    public double setPointDang=0;
    double lasterrorDAng=0;
    double outputDAng=0;
    double IntergralSumDang=0;
    public double PIDcontrollerDang(double kpDang,double kiDang,double kdDang,double setPointDang) {
        this.kpDang = kpDang;
        this.kiDang = kiDang;
        this.kdDang = kdDang;
        this.setPointDang = setPointDang;
        return 0;
    }
    public double calcDang(double MesurementDang){
        double errorDang=setPointDang-MesurementDang;
        double kpDangTerm=kpDang*errorDang;
        IntergralSumDang+=errorDang;
        double kiDangTerm=kiDang*IntergralSumDang;
        double KdDangTerm=(errorDang-lasterrorDAng);
        double KdDangTermA=kdDang*KdDangTerm;
        double outputDAng=kiDangTerm+KdDangTermA+kpDangTerm;
        lasterrorDAng=errorDang;
        return outputDAng;
    }
}


class PIDcontrollerAang{
    public double kpAang=0;
    public double kiAang=0;
    public double kdAang=0;
    public double setPointAang=0;
    double lasterrorAAng=0;
    double outputAAng=0;
    double IntergralSumAang=0;
    public double PIDcontrollerAang(double kpAang,double kiAang,double kdAang,double setPointAang){
        this.kpAang=kpAang;
        this.kiAang=kiAang;
        this.kdAang=kdAang;
        this.setPointAang=setPointAang;
        return 0;
    }
    public double calcAang(double MesurementAang){
        double errorAang=setPointAang-MesurementAang;
        double kpAangTerm=kpAang*errorAang;
        IntergralSumAang+=errorAang;
        double kiAangTerm=kiAang*IntergralSumAang;
        double KdAangTerm=(errorAang-lasterrorAAng);
        double KdAangTermA=kdAang*KdAangTerm;
        double outputAAng=kiAangTerm+KdAangTermA+kpAangTerm;
        lasterrorAAng=errorAang;
        return outputAAng;
    }
}


class PIDcontrollerEang{
    public double kpEang=0;
    public double kiEang=0;
    public double kdEang=0;
    public double setPointEang=0;
    double lasterrorEAng=0;
    double outputEAng=0;
    double IntergralSumEang=0;
    public double PIDcontrollerEang(double kpEang,double kiEang,double kdEang,double setPointEang){
        this.kpEang=kpEang;
        this.kiEang=kiEang;
        this.kdEang=kdEang;
        this.setPointEang=setPointEang;
        return 0;
    }
    public double calcEang(double MesurementEang){
        double errorEang=setPointEang-MesurementEang;
        double kpEangTerm=kpEang*errorEang;
        IntergralSumEang+=errorEang;
        double kiEangTerm=kiEang*IntergralSumEang;
        double KdEangTerm=(errorEang-lasterrorEAng);
        double KdEangTermA=kdEang*KdEangTerm;
        double outputEAng=kiEangTerm+KdEangTermA+kpEangTerm;
        lasterrorEAng=errorEang;
        return outputEAng;
    }
}

class FeedforwardEang{
    public double kSEang=0;//find this value
    public double kAEang=0;//find this value
    public double kVEang=0;//find this value
    double DesiredVEang=0;//find this value
    double DesiredAEang=0;//find this value

    public double feedforwardtermEang(double DesiredVEang,double DesiredAEang,double kSEang,double kVEang,double kAEang){
        this.kSEang=kSEang;
        this.kVEang=kVEang;
        this.kAEang=kAEang;
        double outputffEang=kSEang*Math.abs(DesiredVEang)+kVEang*DesiredVEang+kAEang*DesiredAEang;
        return outputffEang;
    }
}
class FeedforwardDang{
    public double kSDang=0;//find this value
    public double kADang=0;//find this value
    public double kVDang=0;//find this value
    double DesiredVDang=0;//find this value
    double DesiredADang=0;//find this value

    public double feedforwardtermDang(double DesiredVDang,double DesiredADang,double kSDang,double kVDang,double kADang){
        this.kSDang=kSDang;
        this.kVDang=kVDang;
        this.kADang=kADang;
        double outputffDang=kSDang*Math.abs(DesiredVDang)+kVDang*DesiredVDang+kADang*DesiredADang;
        return outputffDang;
    }
}
class FeedforwardAang{
    public double kSAang=0;//find this value
    public double kAAang=0;//find this value
    public double kVAang=0;//find this value
    double DesiredVAang=0;//find this value
    double DesiredAAang=0;//find this value

    public double feedforwardtermAang(double DesiredVAang,double DesiredAAang,double kSAang,double kVAang,double kAAang){
        this.kSAang=kSAang;
        this.kVAang=kVAang;
        this.kAAang=kAAang;
        double outputffAang=kSAang*Math.abs(DesiredVAang)+kVAang*DesiredVAang+kAAang*DesiredAAang;
        return outputffAang;
    }
}
class FeedforwardCang{
    public double kSCang=0;//find this value
    public double kACang=0;//find this value
    public double kVCang=0;//find this value
    double DesiredVCang=0;//find this value
    double DesiredACang=0;//find this value

    public double feedforwardtermCang(double DesiredVCang,double DesiredACang,double kSCang,double kVCang,double kACang){
        this.kSCang=kSCang;
        this.kVCang=kVCang;
        this.kACang=kACang;
        double outputffCang=kSCang*Math.abs(DesiredVCang)+kVCang*DesiredVCang+kACang*DesiredACang;
        return outputffCang;
    }
}
class FeedforwardB{
    public double kSB=0.1;//find this value
    public double kAB=0.1;//find this value
    public double kVB=0.1;//find this value
    double DesiredVB=0.1;//find this value
    double DesiredAB=0.10;//find this value

    public double feedforwardtermB(double DesiredVB,double DesiredAB,double kSB,double kVB,double kAB){
        this.kSB=kSB;
        this.kVB=kVB;
        this.kAB=kAB;
        double outputffB=kSB*Math.abs(DesiredVB)+kVB*DesiredVB+kAB*DesiredAB;
        return outputffB;
    }
}

class PIDCONTOLLERshooterB{
    public double kpshooterB=0.2; //find this value
    public double kishooterB=0.3; //find this value
    public double kdshooterB=0.1;//find this value
    double previousErrorshooterB=0;
    double intergralshooterB=0; //assign a value in the future to intergral
    double minOutputshooterB=-0.5; //assign a value in the future to minoutput
    double maxOutputshooterB=0.5; //assign a value in the future to maxoutput

    public double PIDshooterB(double kpshooterB, double kishooterB, double kdshooterB, double shooterB1, double shooterB2, double shooterB3){

        this.kpshooterB =kpshooterB;
        this.kishooterB =kishooterB;
        this.kdshooterB =kdshooterB;
        double outputshooterBa = kpshooterB * shooterB1 + kishooterB * shooterB3 + kdshooterB * shooterB2;
        return outputshooterBa;
    }
    public double calcshooterB(double targetshooterB, double currentshooterB){
        double errorshooterB = targetshooterB - currentshooterB;
        double integralmaxB=1000;//update this value if needed
        double timeBs=0.2; //update if needed
        double integralshooterB =+ errorshooterB*timeBs;
        if(integralshooterB>integralmaxB){
            integralshooterB=integralmaxB;
        }
        if(integralshooterB<(integralmaxB*-1)){
            integralshooterB=integralmaxB*-1;
        }
        double derivativeshooterB = errorshooterB - previousErrorshooterB/timeBs;
        double outputshooterBa=0;
        outputshooterBa=PIDshooterB(kpshooterB, kishooterB, kdshooterB, errorshooterB, derivativeshooterB, integralshooterB);
        double outputshooterB = Math.max(minOutputshooterB, Math.min(maxOutputshooterB, outputshooterBa));

        double previousErrorshooterB = errorshooterB;
        return outputshooterB;
    }
    public void resetshooterB(){
        double previousErrorshooterB=0;
        double intergralshooterB=0;
    }

}

//frontrightwheel
class PIDCONTOLLERshooterA{


    public double kpshooterA=0.2; //find this value
    public double kishooterA=0.3; //find this value
    public double kdshooterA=0.1;//find this value

    double previousErrorshooterA=0;
    double intergralshooterA=0; //assign a value in the future to intergral
    double minOutputshooterA=-0.5; //assign a value in the future to minoutput
    double maxOutputshooterA=0.5; //assign a value in the future to maxoutput

    public double PIDshooterA(double kpshooterA, double kishooterA, double kdshooterA, double shooterA1, double shooterA2, double shooterA3){

        this.kpshooterA =kpshooterA;
        this.kishooterA =kishooterA;
        this.kdshooterA =kdshooterA;
        double outputshooterAa = kpshooterA * shooterA1 + kishooterA * shooterA3 + kdshooterA * shooterA2;
        return outputshooterAa;
    }
    public double calcshooterA(double targetshooterA, double currentshooterA){
        double errorshooterA = targetshooterA - currentshooterA;

        double integralmaxA=1000;//update this value if needed
        double timeAs=0.2; //update if needed
        double integralshooterA =+ errorshooterA*timeAs;
        if(integralshooterA>integralmaxA){
            integralshooterA=integralmaxA;
        }
        if(integralshooterA<(integralmaxA*-1)){
            integralshooterA=integralmaxA*-1;
        }
        double derivativeshooterA = errorshooterA - previousErrorshooterA/timeAs;
        double outputshooterAa=0;
        outputshooterAa = PIDshooterA(kpshooterA, kishooterA, kdshooterA, errorshooterA, derivativeshooterA, integralshooterA);
        double outputshooterA = Math.max(minOutputshooterA, Math.min(maxOutputshooterA, outputshooterAa));

        double previousErrorshooterA = errorshooterA;
        return outputshooterA;
    }

}
class FeedforwardA{
    double kSA=0.1;//find this value
    double kAA=0.1;//find this value
    double kVA=0.1;//find this value
    double DesiredVA=0.1;//find this value
    double DesiredAA=0.1;//find this value

    public double feedforwardtermA(double DesiredVA,double DesiredAA,double kSA,double kVA,double kAA){
        this.kSA=kSA;
        this.kVA=kVA;
        this.kAA=kAA;
        double outputffA=kSA*Math.abs(DesiredVA)+kVA*DesiredVA+kAA*DesiredAA;
        return outputffA;
    }
}
//frontleftwheel
class PIDCONTOLLERshooterC{


    public double kpshooterC=0.2; //find this value
    public double kishooterC=0.3; //find this value
    public double kdshooterC=0.1;//find this value

    double previousErrorshooterC=0;
    double intergralshooterC=0; //assign a value in the future to intergral
    double minOutputshooterC=-0.5; //assign a value in the future to minoutput
    double maxOutputshooterC=0.5; //assign a value in the future to maxoutput

    public double PIDshooterC(double kpshooterC, double kishooterC, double kdshooterC, double shooterC1, double shooterC2, double shooterC3){

        this.kpshooterC=kpshooterC;
        this.kishooterC=kishooterC;
        this.kdshooterC=kdshooterC;
        double outputshooterCa = kpshooterC * shooterC1 + kishooterC * shooterC3 + kdshooterC * shooterC2;
        return outputshooterCa;
    }
    public double calcshooterC(double targetshooterC, double currentshooterC){
        double errorshooterC = targetshooterC - currentshooterC;

        double integralmaxC=1000;//update this value if needed
        double timeCs=0.2; //update if needed
        double integralshooterC =+ errorshooterC*timeCs;
        if(integralshooterC>integralmaxC){
            integralshooterC=integralmaxC;
        }
        if(integralshooterC<(integralmaxC*-1)){
            integralshooterC=integralmaxC*-1;
        }
        double derivativeshooterC = errorshooterC - previousErrorshooterC/timeCs;
        double outputshooterAC=0;
        outputshooterAC = PIDshooterC(kpshooterC, kishooterC, kdshooterC, errorshooterC, derivativeshooterC, integralshooterC);
        double outputshooterC = Math.max(minOutputshooterC, Math.min(maxOutputshooterC, outputshooterAC));

        double previousErrorshooterC = errorshooterC;
        return outputshooterC;
    }
    public void resetshooterC(){
        double previousErrorshooterC=0;
        double intergralshooterC=0;
    }
}
class FeedforwardC{
    public double kSC=0.1;//find this value
    public double kAC=0.1;//find this value
    public double kVC=0.1;//find this value
    double DesiredVC=0.1;//find this value
    double DesiredAC=0.10;//find this value

    public double feedforwardtermC(double DesiredVC,double DesiredAC,double kSC,double kVC,double kAC){
        this.kSC=kSC;
        this.kVC=kVC;
        this.kAC=kAC;
        double outputffC=kSC*Math.abs(DesiredVC)+kVC*DesiredVC+kAC*DesiredAC;
        return outputffC;
    }
}
//backrightwheel
class PIDCONTOLLERshooterD{


    public double kpshooterD=0.2; //find this value
    public double kishooterD=0.3; //find this value
    public double kdshooterD=0.1;//find this value

    double previousErrorshooterD=0;
    double intergralshooterD=0; //assign a value in the future to intergral
    double minOutputshooterD=-0.5; //assign a value in the future to minoutput
    double maxOutputshooterD=0.5; //assign a value in the future to maxoutput

    public double PIDshooterD(double kpshooterD, double kishooterD, double kdshooterD, double shooterD1, double shooterD2, double shooterD3){

        this.kpshooterD=kpshooterD;
        this.kishooterD=kishooterD;
        this.kdshooterD=kdshooterD;
        double outputshooterDa = kpshooterD * shooterD1 + kishooterD * shooterD3 + kdshooterD * shooterD2;
        return outputshooterDa;
    }
    public double calcshooterD(double targetshooterD, double currentshooterD){
        double errorshooterD = targetshooterD - currentshooterD;

        double integralmaxD=1000;//update this value if needed
        double timeDs=0.2; //update if needed
        double integralshooterD =+ errorshooterD*timeDs;
        if(integralshooterD>integralmaxD){
            integralshooterD=integralmaxD;
        }
        if(integralshooterD<(integralmaxD*-1)){
            integralshooterD=integralmaxD*-1;
        }
        double derivativeshooterD = errorshooterD - previousErrorshooterD/timeDs;
        double outputshooterAD = PIDshooterD(kpshooterD, kishooterD, kdshooterD, errorshooterD, derivativeshooterD, integralshooterD);
        double outputshooterD = Math.max(minOutputshooterD, Math.min(maxOutputshooterD, outputshooterAD));

        double previousErrorshooterD = errorshooterD;
        return outputshooterD;
    }
    public void resetshooterD(){
        double previousErrorshooterD=0;
        double intergralshooterD=0;
    }
}
class FeedforwardD{
    public double kSD=0.1;//find this value
    public double kAD=0.1;//find this value
    public double kVD=0.1;//find this value
    double DesiredVD=0.1;//find this value
    double DesiredAD=0.10;//find this value

    public double feedforwardtermD(double DesiredVD,double DesiredAD,double kSD,double kVD,double kAD){
        this.kSD=kSD;
        this.kVD=kVD;
        this.kAD=kAD;
        double outputffD=kSD*Math.abs(DesiredVD)+kVD*DesiredVD+kAD*DesiredAD;
        return outputffD;
    }
}
//backleftwheel
class PIDCONTOLLERshooterE{


    public double kpshooterE=0.2; //find this value
    public double kishooterE=0.3; //find this value
    public double kdshooterE=0.1;//find this value

    double previousErrorshooterE=0;
    double intergralshooterE=0; //assign a value in the future to intergral
    double minOutputshooterE=-0.5; //assign a value in the future to minoutput
    double maxOutputshooterE=0.5; //assign a value in the future to maxoutput

    public double PIDshooterE(double kpshooterE, double kishooterE, double kdshooterE, double shooterE1, double shooterE2, double shooterE3){

        this.kpshooterE=kpshooterE;
        this.kishooterE=kishooterE;
        this.kdshooterE=kdshooterE;
        double outputshooterEa = kpshooterE * shooterE1 + kishooterE * shooterE3 + kdshooterE * shooterE2;
        return outputshooterEa;
    }
    public double calcshooterE(double targetshooterE, double currentshooterE){
        double errorshooterE = targetshooterE - currentshooterE;

        double integralmaxE=1000;//update this value if needed
        double timeEs=0.2; //update if needed
        double integralshooterE =+ errorshooterE*timeEs;
        if(integralshooterE>integralmaxE){
            integralshooterE=integralmaxE;
        }
        if(integralshooterE<(integralmaxE*-1)){
            integralshooterE=integralmaxE*-1;
        }
        double derivativeshooterE = errorshooterE - previousErrorshooterE/timeEs;
        double outputshooterAE = PIDshooterE(kpshooterE, kishooterE, kdshooterE, errorshooterE, derivativeshooterE, integralshooterE);
        double outputshooterE = Math.max(minOutputshooterE, Math.min(maxOutputshooterE, outputshooterAE));

        double previousErrorshooterE = errorshooterE;
        return outputshooterE;
    }
    public void resetshooterE(){
        double previousErrorshooterE=0;
        double intergralshooterE=0;
    }
}
class FeedforwardE{
    public double kSE=0.1;//find this value
    public double kAE=0.1;//find this value
    public double kVE=0.1;//find this value
    double DesiredVE=0.1;//find this value
    double DesiredAE=0.1;//find this value

    public double feedforwardtermE(double DesiredVE,double DesiredAE,double kSE,double kVE,double kAE){
        this.kSE=kSE;
        this.kVE=kVE;
        this.kAE=kAE;
        double outputffE=kSE*Math.abs(DesiredVE)+kVE*DesiredVE+kAE*DesiredAE;
        return outputffE;
    }
}
class calcPhys{
    public static double answervelocity(double heightofgoal, double g, double theta){
        return (Math.sqrt(2*g*heightofgoal)/Math.sin(theta));
    }
}
class motorintake{
    public double set_intake(double intakepower){
        return intakepower;
    }
}

/**
 * Initialize AprilTag Detection.
 */





/**
 * Initialize AprilTag Detection.
 */
class startup {
    public boolean contains(String stringToSearch, String containText) {
        if (stringToSearch.indexOf(containText) + 1 == 0) {
            return false;
        }
        return true;
    }

    //public void initAprilTag() {
        
        //AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        //VisionPortal.Builder myVisionPortalBuilder;
        //boolean USE_WEBCAM;
        // First, create an AprilTagProcessor.Builder.
        //myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
        //Position cameraPosition;
        //YawPitchRollAngles cameraOrientation;
        //AprilTagProcessor.Builder builder = myAprilTagProcessorBuilder.setCameraPose(cameraPosition, cameraOrientation);
        // Create an AprilTagProcessor by calling build.
        //AprilTagProcessor myAprilTagProcessor = myAprilTagProcessorBuilder.build();
        // Next, create a VisionPortal.Builder and set attributes related to the camera.
        //VisionPortal.Builder setCamera = new VisionPortal.Builder();
        
        //if (USE_WEBCAM==true) {
            // Use a webcam.
          //  VisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        //} else {
            // Use the device's back camera.
            //myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
        //}
        // Add myAprilTagProcessor to the VisionPortal.Builder.
        //myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
        // Create a VisionPortal by calling build.
        //VisionPortal myVisionPortal = myVisionPortalBuilder.build();
    //}
}
class filter{
  public double filterinput(double a,double filterA,double currentinput){
    return (a*filterA)+((1-a)*currentinput);
  }  
}
class Servos_A{
    public static double angles(double desiredangle){
        return Math.abs(desiredangle);
    }
}
class Servos_B{
    public static double direction(double powerofintake){
        return powerofintake;
    }
}
