package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import java.util.concurrent.TimeUnit;
import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import java.lang.Object;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous((name = "Sensor: IMU Non-OrthogonalA", group = "Sensor")
public class W_nonorthoA extends LinearOpMode {
  double objectA=0;
  
  double bearingangle=0;
  double currentheading=0;
  IMU imu;
  double deltaXa=0;
  double deltaYa=0;
  double bearingofbot=0;
  double currentrangeofbot=0;
  double bearingoftag=0;
  double deltaX=0;
  double startx=0;
  double starty=0;
  double deltaY=0;
  double bearingB=0;
  double ZA=0;
  double XA=0;
  double YA=0;
  double cicumferenceofwheel=104*Math.PI;
  public double anglestartA=35.89;//degrees change angle if neccesary //red basket
  //public double anglestartB=;//degrees change angle if neccesary //robot start on around white line of start
  public double Circumference=76.8*Math.PI; //in mm
  double objectB=0;
  double objectC=0;
  double sumofobjects=5;
  boolean USE_WEBCAM;
  AprilTagProcessor myAprilTagProcessor;
  Position cameraPosition;
  YawPitchRollAngles cameraOrientation;
  VisionPortal myVisionPortal;
  private NormalizedColorSensor test_color;
  private NormalizedColorSensor test_colorA;
  double motifs=0;
  double targets=0;
  ExposureControl myExposureControl;
  long minExposure;
  long maxExposure;
  double angletarget=0;
  GainControl myGainControl;
  double myExposure;
  int minGain;
  int maxGain;
  double myGain;
  double onerevA=537.7;
  double onerev=384;
  public double rps=435/60;
  double rangeA=0;
  double rangeofrobot=0;
  double bearingA=0;
  double servostatusA=0;
  double servostatusB=0;
  double servoestatusC=0;
  double servoestatusD=0;
  double servoestatusE=0;
  double desiredvelocity=1305;
  double desiredspeed=0;
  double intakestatus=0;
  double intakespeed=0;
  public double intialspeed=0; //intial speed before change measured in ticks adjust value
  public double degree1=0.5;
  public double degree2=0;
  public double latchopen=0.5; 
  public double latchclose=0;
  public double offsetX=9.129397076417323;
  public double offsetY=5.1496063;//fine tune this
  double pointAx=-16.5354;//tune this in inches times -1
  double pointBx=0.0001;
  double pointCx=0;
  double pointAy=0.0001;
  double pointBy=16.5354;// robot height of camera in inches
  double pointCy=38.759843+offsetY;
  double A=0;
  double B=0;
  double C=0;
  double rangeB=0;
  public double gravity=386.08858267717;
  public double artifactholderopen=0.5; // adjust value in the future
  public double artifactholderclose=0; // adjust value in the future
  public double shooterholderopen=0.5; // adjust value in the future
  public double shooterholderclose=0; // adjust value in the future 
  public double beltspeed1=-1; // adjust value in the future
  public double beltspeed2=1; // adjust value in the future
  private DcMotor backleft;
  private DcMotor backright;
  private DcMotor frontright;
  private DcMotor frontleft;
  private DcMotor X;
  private DcMotor intake;
  private Servo pivotintake;
  private Servo pivotintakeA;
  private Servo shooterholder;
  private Servo artifactholder;
  private CRServo belt;
  private CRServo holder;
  private DcMotor shooterwheelA;
  private DcMotor shooterwheelB; 
  public double sign(double v){
         if(v>0){
             return 1; 
         }
         if(v==0){
             return 0;
         }
         if(v<0){
            return -1;
         }
  }
  public double ball(double x1, double y1, double x2, double y2, double state){
       double c=(x2 * x3 * (x2 - x3) * y1 + x3 * x1 * (x3 - x1) * y2 + x1 * x2 * (x1 - x2) * y3)/((x3*x3 - x2*x2) * (x2 - x1) - (x2*x2 - x1*x1) * (x3 - x2));
       double a = ((y3 - y2) * ((x2 - x1) - (y2 - y1)) * (x3 - x2)) / ((x3*x3 - x2*x2) * (x2 - x1) - (x2*x2 - x1*x1) * (x3 - x2));
       double b=0;
       if (Math.abs(x2 - x1) > 1e-9) {
            b = ((y2 - y1) - a * (x2*x2 - x1*x1)) / (x2 - x1);
       } else {
            b = ((y3 - y2) - a * (x3*x3 - x2*x2)) / (x3 - x2);
       }
       if(state==1){
           return a;
       }
       if(state==2){
           return c;
       }
       if(state==3){
           return b;
       }

  }
  public class PIDCONTOLLERFL{
    


    public double kpFL; //find value
    public double kiFL; //find value
    public double kdFL;//find value
    
    double previousErrorFL=0;
    double intergralFL=0; //assign a value in the future to intergral
    double minOutputFL=0; //assign a value in the future to minoutput
    double maxOutputFL=0; //assign a value in the future to maxoutput
    public double PIDFL(double kpFL, double kiFL, double kdFL, double FL1, double FL2, double FL3){
       
       this.kpFL=kpFL;
       this.kiFL=kiFL;
       this.kdFL=kdFL;
       double outputFLa = kpFL * FL1 + kiFL * FL3 + kdFL * FL2;
       return outputFLa;
    }
    public double calcFL(double targetFL,double currentFL){
        double errorFL = targetFL - currentFL;
        double integralFL =+ errorFL;
        double derivativeFL = errorFL - previousErrorFL;
        double outputFLa = PIDFL(kpFL, kiFL, kdFL, errorFL, derivativeFL, integralFL);
        double outputFL = Math.max(minOutputFL, Math.min(maxOutputFL, outputFLa));

        double previousErrorFL = errorFL;
        return outputFL;
    }
    public void resetFL(){
       double previousErrorFL=0;
       double intergralFL=0;
    }
 }
public double normalizecurrentangle(double angle){
    double turingangle=angle % 360; //try this if not working
  if(turningangleA>=180){
     turningangleA-=360;
     return turningangleA;
     
  } else if(turningangleA<-180){
     turningangleA+=360;
     return turningangleA;
  }
}
public double bearingAngleFound(double cheading,double bangle){
  double turingangle=(bangle-cheading) % 360; //try this if not working
  if(turningangle>=180){
     turningangle-=360;
     return turningangle;
     
  } else if(turningangle<-180){
     turningangle+=360;
     return turningangle;
  }
}
public class PIDCONTOLLERbearing{
    


    public double kpbearing; //find value
    public double kibearing; //find value
    public double kdbearing;//find value
    
    double previousErrorbearing=0;
    double intergralbearing=0; //assign a value in the future to intergral
    double minOutputbearing=0; //assign a value in the future to minoutput
    double maxOutputbearing=0; //assign a value in the future to maxoutput
    public double PIDbearing(double kpbearing, double kibearing, double kdbearing, double bearing1, double bearing2, double bearing3){
       
       this.kpbeaing=kpbearing;
       this.kibearing=kibearing;
       this.kdbearing=kdbearing;
       double outputbearinga = kpbearing * bearing1 + kibearing * bearing3 + kdbearing * bearing2;
       return outputbearinga;
    }
    public double calcbearing(double targetbearing,double currentbearing){
        double errorbearing=(targetbearing-currentbearing)%360;
        if(errorbearing>=180){
            erorrbearing-=360;
          }  
        } else if(errorbearing<-180){
             errorbearing+=360;
        }
        double integralbearing  =+ errorbearing;
        double derivativebearing  = errorbearing - previousErrorbearing;
        double outputbearinga = PIDbearing(kpbearing, kibearing, kdbearing, errorbearing, derivativebearing, integralbearing);
        double outputbearing  = Math.max(minOutputbearing, Math.min(maxOutputbearing , outputbearinga));

        double previousErrorbearing = errorbearing;
        return outputbearing;
    }
    public void resetbearing(){
       double previousErrorbearing=0;
       double intergralbearing=0;
    }
 }
 public class PIDCONTOLLERrange{
    


    public double kprange; //find value
    public double kirange; //find value
    public double kdrange;//find value
    
    double previousErrorrange=0;
    double intergralrange=0; //assign a value in the future to intergral
    double minOutputrange=0; //assign a value in the future to minoutput
    double maxOutputrange=0; //assign a value in the future to maxoutput
    public double PIDrange(double kprange, double kirange, double kdrange, double range1, double range2, double range3){
       
       this.kprange=kprange;
       this.kirange=kirange;
       this.kdrange=kdrange;
       double outputrangea = kprange * range1 + kirange * range3 + kdrange * range2;
       return outputbearinga;
    }
    public double calcrange(double targetrange,double currentrange){
        double errorrange = targetrange - currentrange;
        double integralrange  =+ errorrange;
        double derivativerange  = errorrange - previousErrorrange;
        double outputrangea = PIDrange(kprange, kirange, kdrange, errorrange, derivativerange, integralrange);
        double outputrange  = Math.max(minOutputrange, Math.min(maxOutputrange , outputrangea));

        double previousErrorrange = errorrange;
        return outputrange;
    }
    public void resetrange(){
       double previousErrorrange=0;
       double intergralrange=0;
    }
 }
 public class PIDCONTOLLERyaw{
    


    public double kpyaw; //find value
    public double kiyaw; //find value
    public double kdyaw;//find value
    
    double previousErroryaw=0;
    double intergralyaw=0; //assign a value in the future to intergral
    double minOutputyaw=0; //assign a value in the future to minoutput
    double maxOutputyaw=0; //assign a value in the future to maxoutput
    public double PIDyaw(double kpyaw, double kiyaw, double kdyaw, double yaw1, double yaw2, double yaw3){
       
       this.kpyaw=kpyaw;
       this.kiyaw=kiyaw;
       this.kdyaw=kdyaw;
       double outputyawa = kpyaw * yaw1 + kiyaw * yaw3 + kdyaw * yaw2;
       return outputyawa;
    }
    public double calcyaw(double targetyaw,double currentyaw){
        double erroryaw = (targetyaw - currentyaw)%360;
        if(erroryaw>=180){
            erorryaw-=360;
          }  
        } else if(erroryaw<-180){
             erroryaw+=360;
        }
        double integralyaw  =+ erroryaw;
        double derivativeyaw  = erroryaw - previousErroryaw;
        double outputyawa = PIDyaw(kpyaw, kiyaw, kdyaw, erroryaw, derivativeyaw, integralyaw);
        double outputyaw  = Math.max(minOutputyaw, Math.min(maxOutputyaw , outputyawa));

        double previousErroryaw = erroryaw;
        return outputyaw;
    }
    public void resetyaw(){
       double previousErroryaw=0;
       double intergralyaw=0;
    }
 }
 public class PIDCONTOLLERFR{
    public double kpFR; //find value
    public double kiFR;//find value
    public double kdFR;//find value
    double previousErrorFR=0;
    double maxOutputFR=0; //assign a value in the future to maxoutput
    double intergralFR=0; //assign a value in the future to intergral
    double minOutputFR=0; //assign a value in the future to minoutput

    public double PIDFR(double kpFR, double kiFR, double kdFR, double FR1, double FR2, double FR3){
       this.kpFR=kpFR;
       this.kiFR=kiFR;
       this.kdFR=kdFR;
       double outputFRa = kpFR * FR1 + kiFR * FR3 + kdFR * FR2;
       return outputFRa;
    }
    public double calcFR(double targetFR,double currentFR){
        double errorFR = targetFR - currentFR;
        double integralFR =+ errorFR;
        double derivativeFR = errorFR - previousErrorFR;
        double outputFRa = kpFR * errorFR + kiFR * integralFR + kdFR * derivativeFR;
        double outputFR = Math.max(minOutputFR, Math.min(maxOutputFR, outputFRa));

        double previousErrorFR = errorFR;
        return outputFR;
    }
    public void resetFR(){
       double previousErrorFR=0;
       double intergralFR=0;
    }
 }

 public class PIDCONTOLLERBR{
    public double kpBR;//find value
    public double kiBR; //find value
    public double kdBR; //find value
    double previousErrorBR=0;
    double maxOutputBR=0; //assign a value in the future to maxoutput
    double intergralBR=0; //assign a value in the future to intergral
    double minOutputBR=0; //assign a value in the future to minoutput
    public double PIDBR(double kpBR, double kiBR, double kdBR, double BR1, double BR3, double BR2){
       this.kpBR=kpBR;
       this.kiBR=kiBR;
       this.kdBR=kdBR;
       double outputBRa = kpBR * BR1 + kiBR * BR3 + kdBR * BR2;
       return outputBRa;
    }
    public double calcBR(double targetBR,double currentBR){
        double errorBR = targetBR - currentBR;
        double integralBR =+ errorBR;
        double derivativeBR = errorBR - previousErrorBR;
        double outputBRa1=PIDBR(kpBR, kiBR, kdBR, errorBR, integralBR, derivativeBR);
        double outputBR = Math.max(minOutputBR, Math.min(maxOutputBR, outputBRa1));

        double previousErrorBR = errorBR;
        return outputBR;
    }
    public void resetBR(){
       double previousErrorBR=0;
       double intergralBR=0;
    }
 }  
 public class PIDCONTOLLERBL{
    


    public double kpBL; //find value
    public double kiBL; //find value
    public double kdBL;//find value
    double previousErrorBL=0;
    double intergralBL=0; //assign a value in the future to intergral
    double minOutputBL=0; //assign a value in the future to minoutput
    double maxOutputBL=0; //assign a value in the future to maxoutput
    public double PIDFL(double kpBL, double kiBL, double kdBL, double BL1, double BL3, double BL2){
       this.kpBL=kpBL;
       this.kiBL=kiBL;
       this.kdBL=kdBL;
       double outputBLa = kpBL * BL1 + kiBL * BL3 + kdBL * BL2;
       return outputBLa;
    }
    public double calcBL(double targetBL,double currentBL){
        double errorBL = targetBL - currentBL;
        double integralBL =+ errorBL;
        double derivativeBL = errorBL - previousErrorBL;
        double outputBLa = PIDFL(kpBL, kiBL, kdBL, errorBL, derivativeBL, integralBL);
        double outputBL = Math.max(minOutputBL, Math.min(maxOutputBL, outputBLa));

        double previousErrorBL = errorBL;
        return outputBL;
    }
    public void resetBL(){
       double previousErrorBL=0;
       double intergralBL=0;
    }
  }

   public class FeedforwardB{
      public double kSB;//find value
      public double kAB;//find value
      public double kVB;//find value
      double DesiredVB=1305;
      double DesiredAB=0;//find this value
      public class FeedforwardB{
       public double feedforwardtermB(double DesiredVB,double DesiredAB,double kSB,double kVB,double kAB){
           this.kSB=kSB;
           this.kVB=kVB;
           this.kAB=kAB;
           double outputffB=kSB*sign(DesiredVB)+kVB*DesiredVB+kAB*DesiredAB;
           return outputffB;
          }
      }
  public class PIDCONTOLLERshooterB{
      public double kpshooterB; //find value
      public double kishooterB; //find value
      public double kdshooterB;//find value
      double previousErrorshooterB=0;
      double intergralshooterB=0; //assign a value in the future to intergral
      double minOutputshooterB=0; //assign a value in the future to minoutput
      double maxOutputshooterB=0; //assign a value in the future to maxoutput
      public class PIDCONTOLLERshooterB{
       public double PIDshooterB(double kpshooterB, double kishooterB, double kdshooterB, double shooterB1, double shooterB2, double shooterB3){
    
         this.kpshooterB=kpshooterB;
         this.kishooterB=kishooterB;
         this.kdshooterB=kdshooterB;
         double outputshooterBa = kpshooterB * shooterB1 + kishooterB * shooterB3 + kdshooterB * shooterB2;
         return outputshooterBa;
       }
       public double calcshooterB(double targetshooterB,double currentshooterB){
          double errorshooterB = targetshooterB - currentshooterB;
          double integralmaxB=1000;//update this value if needed
          double timeBs=0.2; //update if needed
          double integralshooterB =+ errorshooterB*timeBs;
          if(integralshooterB>integralmaxB){
              integralshooterB=integralmaxB;
          if(integralshooterB<-integralmaxB){
              integralshooterB=integralmaxB*-1;
          }
          double derivativeshooterB = errorshooterB - previousErrorshooterB/timeBs;
          double outputshooterBa = PIDshooterB(kpshooterB, kishooterB, kdshooterB, errorshooterB, derivativeshooterB, integralshooterB);
          double outputshooterB = Math.max(minOutputshooterB, Math.min(maxOutputshooterB, outputshooterBa));
  
          double previousErrorshooterB = errorshooterB;
          return outputshooterB;
      }
      public void resetshooterB(){
         double previousErrorshooterB=0;
         double intergralshooterB=0;
      }

    }
   public class FeedforwardA{
      public double kSA;//find value
      public double kAA;//find value
      public double kVA;//find value
      double DesiredVA=1305; //change if needed
      double DesiredAA=0;//find this value
      public class FeedforwardA{
       public double feedforwardtermA(double DesiredVA,double DesiredAA,double kSA,double kVA,double kAA){
           this.kSA=kSA;
           this.kVA=kVA;
           this.kAA=kAA;
           double outputffA=kSA*sign(DesiredVA)+kVA*DesiredVA+kAA*DesiredAA;
           return outputffA;
          }
      }
    public class PIDCONTOLLERshooterA{
      

      public double kpshooterA; //find value
      public double kishooterA; //find value
      public double kdshooterA; //find value
      
      double previousErrorshooterA=0;
      double intergralshooterA=0; //assign a value in the future to intergral
      double minOutputshooterA=0; //assign a value in the future to minoutput
      double maxOutputshooterA=0; //assign a value in the future to maxoutput
      public class PIDCONTOLLERshooterA{
       public double PIDshooterA(double kpshooterA, double kishooterA, double kdshooterA, double shooterA1, double shooterA2, double shooterA3){
         
         this.kpshooterA=kpshooterA;
         this.kishooterA=kishooterA;
         this.kdshooterA=kdshooterA;
         double outputshooterAa = kpshooterA * shooterA1 + kishooterA * shooterA3 + kdshooterA * shooterA2;
         return outputshooterAa;
       }
       public double calcshooterA(double targetshooterA,double currentshooterA){
          double errorshooterA = targetshooterA - currentshooterA;
          
          double integralmaxA=1000;//update this value if needed
          double timeAs=0.2; //update if needed
          double integralshooterA =+ errorshooterA*timeAs;
          if(integralshooterA>integralmaxA){
              integralshooterA=integralmaxA;
          if(integralshooterA<-integralmaxA){
              integralshooterA=integralmaxA*-1;
          }
          double outputshooterAa = PIDshooterA(kpshooterA, kishooterA, kdshooterA, errorshooterA, derivativeshooterA, integralshooterA);
          double outputshooterA = Math.max(minOutputshooterA, Math.min(maxOutputshooterA, outputshooterAa));
  
          double previousErrorshooterA = errorshooterA;
          return outputshooterA;
      }    
      public void resetshooterA(){
         double previousErrorshooterA=0;
         double intergralshooterA=0;
      }
    }
    public double rangeAcontrol(double rangeCcontrol, double rangeDcontrol){
      PIDCONTOLLERrange rangeA=new PIDCONTOLLERrange();
      double powerRange=rangeA.calcrange(rangeDcontrol,rangeCcontrol);
      return powerRange;
    }
    public double YawAcontrol(double YawCcontrol, double YawDcontrol){
      PIDCONTOLLERyaw yawA=new PIDCONTOLLERyaw();
      double powerYaw=yawA.calcyaw(YawDcontrol,YawCcontrol);
      return powerYaw;
    }
    public double BearingAcontrol(double BearingCcontrol, double BearingDcontrol){
      PIDCONTOLLERbearing bearingA=new PIDCONTOLLERbearing();
      
      double powerBearing=bearingA.calcbearing(BearingDcontrol,BearingCcontrol);
      return powerBearing;
    }
    public double BLA(double BLC,double BLD){
      PIDCONTOLLERBL BL=new PIDCONTOLLERBL();
      double powerBLA=BL.calcBL(BLC,BLD);
      return powerBLA;
    }
    public double FLA(double FLC,double FLD){
      PIDCONTOLLERFL FL=new PIDCONTOLLERFL();
      double powerFLA=FL.calcFL(FLC,FLD);
      return powerFLA;
    }
    public double FRA(double FRC,double FRD){
      PIDCONTOLLERFR FR=new PIDCONTOLLERFR();
      double powerFRA=FR.calcFR(FRC,FRD);
      return powerFRA;
    }
    public double BRA(double BRC,double BRD){
      PIDCONTOLLERBR BR=new PIDCONTOLLERBR();
      double powerBRA=BR.calcBR(BRC,BRD);
      return powerBRA;
    }
    public double shooterB(double SBC, double SBD, double SBVB, double SBAB, double SBkSA,double SBkVA,double SBkAA){
        PIDCONTOLLERshooterB ShooterB=PIDCONTOLLERshooterB();
        feedforwardtermB termB=feedforwardtermB();
        double speedofShooterB=ShooterB.calcshooterB(SBD,SBC);
        double feedforwardB=termB.feedforwardtermB(SBVB,SBAB,SBkSA,SBkVA,SBkAA);
        double finalspeedofshooterB=speedofShooterB+feedforwardB;
        return finalspeedofshooterB;
    }
    public double shooterA(double SAC, double SAD, double SAVB, double SAAB, double SAkSA,double SAkVA,double SAkAA){
        PIDCONTOLLERshooterA ShooterA=PIDCONTOLLERshooterA();
        double speedofShooterA=ShooterB.calcshooterB(SAD,SAC);
        double feedforwardA=termA.feedforwardtermB(SAVB,SAAB,SAkSA,SAkVA,SAkAA);
        double finalspeedofshooterA=speedofShooterA+feedforwardA;
        return finalspeedofshooterA;
    }
    public double intakeObjects(double status){
       if(status==1){
          return 0.8;
       }
       else{
         return 0;
       }
    }
    public double servoA(double stateA){
       if(stateA==1){
         return 0.5; 
       }
       else{
         return 0;
       }  
    }
    public double servoB(double stateB){
       if(stateB==1){
         return 0.5; 
       }
       else{
         return 0;
       }  
    }    
    public double servoC(double stateC){
       if(stateC==1){
         return 1; 
       }
       if(stateC==-1){
         return -1;
       }
       if(stateC==0){
         return 0;
       }  
    }
    public double servoD(double stateD){
       if(stateD==1){
         return 0.5; 
       }
       else{
         return 0;
       }  
    } 
    public double servoE(double stateE){
       if(stateE==1){
         return 0.5; 
       }
       else{
         return 0;
       }  
    } 
  @Override
  public void runOpMode() throws InterruptedException{
    private ElapsedTime runtime = new ElapsedTime();
    imu = hardwareMap.get(IMU.class, "imu");
    pivotintake = hardwareMap.get(Servo.class, "pivot intake");
    pivotintakeA = hardwareMap.get(Servo.class, "pivot intakeA");
    shooterholder= hardwareMap.get(Servo.class, "shooterholder");
    belt = hardwareMap.get(CRServo.class, "belt");
    holder = hardwareMap.get(CRServo.class, "holder");
    artifactholder= hardwareMap.get(Servo.class, "artifactholder");
    shooterwheelA = hardwareMap.get(DcMotor.class, "shooterwheelA");
    shooterwheelB = hardwareMap.get(DcMotor.class, "shooterwheelB");
    intake=hardwareMap.get(DcMotor.class, "intake");
    X=hardwareMap.get(DcMotor.class, "odometrywheelone");
    backleft = hardwareMap.get(DcMotor.class, "backleft");
    backright = hardwareMap.get(DcMotor.class, "backright");
    frontright = hardwareMap.get(DcMotor.class, "frontright");
    frontleft = hardwareMap.get(DcMotor.class, "frontleft");
    shooterwheelA.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    shooterwheelB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    shooterwheelB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    shooterwheelA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    X.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    backright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    X.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    frontleft.setDirection(DcMotor.Direction.REVERSE);
    backright.setDirection(DcMotor.Direction.REVERSE);


    USE_WEBCAM = true;
    test_color = hardwareMap.get(NormalizedColorSensor.class, "test_color");
    test_colorA=hardwareMap.get(NormalizedColorSensor.class, "test_colorA");
    double[] motif={0,0,0,0};
    double [] solutions={0,0,0,0};
    double[] correctmotif={0,0,0,0};
    double red=0;
    double green=0;
    double blue=0;
    double test=0;
    double greenA=0;
    double blue=0;
    double blueA=0;
    double sense=0;
    double senseA=0;
    double X=0;
    double Y=0;
    double Z=0;
    double Pitch=0;
    double Yaw=0;
    double Roll=0;
    double Ycenter=0;
    double Xcenter=0;
    int i=0;
    // Variables to store the position and orientation of the camera on the robot. Setting these
    // values requires a definition of the axes of the camera and robot:
    // Camera axes:
    // Origin location: Center of the lens
    // Axes orientation: +x right, +y down, +z forward (from camera's perspective)
    // Robot axes (this is typical, but you can define this however you want):
    // Origin location: Center of the robot at field height
    // Axes orientation: +x right, +y forward, +z upward
    // Position:
    // If all values are zero (no translation), that implies the camera is at the center of the
    // robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
    // inches above the ground - you would need to set the position to (-5, 7, 12).
    // Orientation:
    // If all values are zero (no rotation), that implies the camera is pointing straight up. In
    // most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
    // the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
    // it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
    // to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
    cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);
    // Initialize AprilTag before waitForStart.
    initAprilTag();
    // Wait for the match to begin.
    telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
    telemetry.addData(">", "Touch START to start OpMode");
    telemetry.update();
    getCameraSetting();
    myExposure = 10;
    myGain = 50;
    double xRotation = 0;  
    double yRotation = 0;  
    double zRotation = 0;  

    Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);


    RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
    imu.initialize(new IMU.Parameters(orientationOnRobot));

    waitForStart();
    runtime.reset();
    while (opModeIsActive()) {
      telemetry.addData("Hub orientation", "X=%.1f,  Y=%.1f,  Z=%.1f \n", xRotation, yRotation, zRotation);
      imu.resetYaw();
      YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
      AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);

      telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", orientation.getYaw(AngleUnit.DEGREES));
      telemetry.addData("Pitch (X)", "%.2f Deg.", orientation.getPitch(AngleUnit.DEGREES));
      telemetry.addData("Roll (Y)", "%.2f Deg.\n", orientation.getRoll(AngleUnit.DEGREES));
      telemetry.addData("Yaw (Z) velocity", "%.2f Deg/Sec", angularVelocity.zRotationRate);
      telemetry.addData("Pitch (X) velocity", "%.2f Deg/Sec", angularVelocity.xRotationRate);
      telemetry.addData("Roll (Y) velocity", "%.2f Deg/Sec", angularVelocity.yRotationRate);
      double currentpositionY=intake.getCurrentPosition();
      double currentpositionX=X.getCurrentPosition();    
      backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      double currentpositionBL=backleft.getCurrentPosition();
      double currentpositionBR=backright.getCurrentPosition();
      double currentpositionFR=frontright.getCurrentPosition();
      double currentpositionFL=frontleft.getCurrentPosition();

      double powerBL=BLA(currentpositionBL,-1254);
      double powerFL=FLA(currentpositionFL,-1254);
      double powerFR=FRA(currentpositionFR,1254);
      double powerBR=BRA(currentpositionBR,1254);
      frontright.setPower(powerFR);       
      frontleft.setPower(powerFL);         
      backleft.setPower(powerBL);            
      backright.setPower(powerBR);
      backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      currentpositionBL=backleft.getCurrentPosition();
      currentpositionBR=backright.getCurrentPosition();
      currentpositionFR=frontright.getCurrentPosition();
      currentpositionFL=frontleft.getCurrentPosition();

      powerBL=BLA(currentpositionBL,1515);//change sign if necessary
      powerFL=FLA(currentpositionFL,1515);//change sign if necessary
      powerFR=FRA(currentpositionFR,1515);//change sign if necessary
      powerBR=BRA(currentpositionBR,1515);//change sign if necessary
      frontright.setPower(powerFR);       
      frontleft.setPower(powerFL);         
      backleft.setPower(powerBL);            
      backright.setPower(powerBR);
      backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      currentpositionBL=backleft.getCurrentPosition();
      currentpositionBR=backright.getCurrentPosition();
      currentpositionFR=frontright.getCurrentPosition();
      currentpositionFL=frontleft.getCurrentPosition();
      powerBL=BLA(currentpositionBL,1755);
      powerFL=FLA(currentpositionFL,-1755);
      powerFR=FRA(currentpositionFR,1755);
      powerBR=BRA(currentpositionBR,-1755);
      frontright.setPower(powerFR);       
      frontleft.setPower(powerFL);         
      backleft.setPower(powerBL);            
      backright.setPower(powerBR);
      intakestatus=1;
      intakespeed=intakeObjects(intakestatus);
      intake.setPower(intakespeed);
      holder.setPower(intakespeed);
      backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      currentpositionBL=backleft.getCurrentPosition();
      currentpositionBR=backright.getCurrentPosition();
      currentpositionFR=frontright.getCurrentPosition();
      currentpositionFL=frontleft.getCurrentPosition();
      powerBL=BLA(currentpositionBL,-100);
      powerFL=FLA(currentpositionFL,-100);
      powerFR=FRA(currentpositionFR,100);
      powerBR=BRA(currentpositionBR,100);
      frontright.setPower(powerFR);       
      frontleft.setPower(powerFL);         
      backleft.setPower(powerBL);            
      backright.setPower(powerBR);
      backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      currentpositionBL=backleft.getCurrentPosition();
      currentpositionBR=backright.getCurrentPosition();
      currentpositionFR=frontright.getCurrentPosition();
      currentpositionFL=frontleft.getCurrentPosition();
      powerBL=BLA(currentpositionBL,100);
      powerFL=FLA(currentpositionFL,100);
      powerFR=FRA(currentpositionFR,-100);
      powerBR=BRA(currentpositionBR,-100);
      frontright.setPower(powerFR);       
      frontleft.setPower(powerFL);         
      backleft.setPower(powerBL);            
      backright.setPower(powerBR);
      intakestatus=0;
      intakespeed=intakeObjects(intakestatus);
      intake.setPower(intakespeed);
      holder.setPower(intakespeed);
      backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      currentpositionBL=backleft.getCurrentPosition();
      currentpositionBR=backright.getCurrentPosition();
      currentpositionFR=frontright.getCurrentPosition();
      currentpositionFL=frontleft.getCurrentPosition();
      powerBL=BLA(currentpositionBL,-1755);
      powerFL=FLA(currentpositionFL,1755);
      powerFR=FRA(currentpositionFR,-1755);
      powerBR=BRA(currentpositionBR,1755);
      frontright.setPower(powerFR);       
      frontleft.setPower(powerFL);         
      backleft.setPower(powerBL);            
      backright.setPower(powerBR);
      backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      currentpositionBL=backleft.getCurrentPosition();
      currentpositionBR=backright.getCurrentPosition();
      currentpositionFR=frontright.getCurrentPosition();
      currentpositionFL=frontleft.getCurrentPosition();
      powerBL=BLA(currentpositionBL,-4759);//change sign if needed
      powerFL=FLA(currentpositionFL,-4759);//change sign if needed
      powerFR=FRA(currentpositionFR,-4759);//change sign if needed
      powerBR=BRA(currentpositionBR,-4759);//change sign if needed
      frontright.setPower(powerFR);       
      frontleft.setPower(powerFL);         
      backleft.setPower(powerBL);            
      backright.setPower(powerBR);
      backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      currentpositionBL=backleft.getCurrentPosition();
      currentpositionBR=backright.getCurrentPosition();
      currentpositionFR=frontright.getCurrentPosition();
      currentpositionFL=frontleft.getCurrentPosition();
      powerBL=BLA(currentpositionBL,-1254);//change sign if needed
      powerFL=FLA(currentpositionFL,-1254);//change sign if needed
      powerFR=FRA(currentpositionFR,1254);//change sign if needed
      powerBR=BRA(currentpositionBR,1254);//change sign if needed
      frontright.setPower(powerFR);       
      frontleft.setPower(powerFL);         
      backleft.setPower(powerBL);            
      backright.setPower(powerBR);
      backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      currentpositionBL=backleft.getCurrentPosition();
      currentpositionBR=backright.getCurrentPosition();
      currentpositionFR=frontright.getCurrentPosition();
      currentpositionFL=frontleft.getCurrentPosition();
      powerBL=BLA(currentpositionBL,-251);//change sign if needed
      powerFL=FLA(currentpositionFL,251);//change sign if needed
      powerFR=FRA(currentpositionFR,-251);//change sign if needed
      powerBR=BRA(currentpositionBR,251);//change sign if needed
      frontright.setPower(powerFR);       
      frontleft.setPower(powerFL);         
      backleft.setPower(powerBL);            
      backright.setPower(powerBR);
      List<AprilTagDetection> myAprilTagDetections;
      AprilTagDetection myAprilTagDetection;
      myAprilTagDetections = myAprilTagProcessor.getDetections();
      telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
    // Iterate through list and call a function to display info for each recognized AprilTag.
      for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
        myAprilTagDetection = myAprilTagDetection_item;
      // Display info about the detection.
        telemetry.addLine("");
        if (myAprilTagDetection.metadata != null) {
          telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
         // Only use tags that don't have Obelisk in them since Obelisk tags don't have valid location data
          test=myAprilTagDetection.id;
          motifs=test;
          
          if (!contains(myAprilTagDetection.metadata.name, "Obelisk")) {
            Y=Math.round(myAprilTagDetection.robotPose.getPosition().y*10);
            X=Math.round(myAprilTagDetection.robotPose.getPosition().x*10);
            Z=Math.round(myAprilTagDetection.robotPose.getPosition().z*10);
            Pitch=Math.round(myAprilTagDetection.robotPose.getOrientation().getPitch()*10);
            Roll=Math.round(myAprilTagDetection.robotPose.getOrientation().getRoll()*10);
            Yaw=Math.round(myAprilTagDetection.robotPose.getOrientation().getYaw()*10);
            
            //telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().z, 6, 1) + "  (inch)");
            telemetry.addLine("XYZ " + X/10 + " " + Y/10 + " " + Z/10 + "  (inch)");
            telemetry.addLine("PRY " +  Pitch/10 + " " + Roll/10 + " " + Yaw/10 + " \u03B8 (deg)");
            
          }
        } else {
           telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
           telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
          }

      }
      telemetry.addLine("");
      telemetry.addLine("key:");
      telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
      telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
      if(motifs!=20 && motifs!=24){
       if(motifs==23){
         correctmotif[0]=2;
         correctmotif[1]=2;
         correctmotif[2]=1;
       }
       if(motifs==21){
        correctmotif[0]=1;
        correctmotif[1]=2;
        correctmotif[2]=2;
       }
       if(motifs==22){
        correctmotif[0]=2;
        correctmotif[1]=1;
        correctmotif[2]=2;
       }
      
      // Push telemetry to the Driver Station.
      //sleep(500);
        NormalizedRGBA colors = test_color.getNormalizedColors();
        NormalizedRGBA colorA = test_color.getNormalizedColorA();
        red=colors.red*10;
        green=colors.green*10;
        blue=colors.blue*10;
        blueA=colorA.blue*10;
        redA=colorA.red*10;
        greenA=colorA.green*10;
       //telemetry.addData("blue","3%f",colors.blue);
       //telemetry.addData("green","3%f",colors.green);
        sense=0;
        senseA=0;
        if(blue>=0.05){
                sense=2;

                motif[i]=sense;

                i=i+1;
                   
                
        }
        if(green>=0.05 && blue<0.05){
                //telemetry.addLine("got green artifact");
                sense=1;
                motif[i]=sense;
                i=i+1;
                  
        }
        if(blueA>=0.05 && i!=2 && sense!=1 || sense!=2){
            senseA=2;
            motif[i]=senseA;
            i=i+1;
            
        }
        if(greenA>=0.05 && blueA<0.05 && i!=2 && senseA!=1 || senseA!=2){
            senseA=1;
            motif[i]=senseA;
            i=i+1;
        }
        if(i==2){
           objectC=sumofobjects-(objectA-objectB);
           motif[i]=objectC;
           i=i+1;
        }
        sleep(200); // change delay for cycle of ball to sensor
        if(i==3){
          i=0;
        }
      
        if(correctmotif[0]==motif[0] && motifs!=0){
          telemetry.addLine("first ball is correct");
        }
        else{
          telemetry.addLine("first ball is incorrect");
        }
        if(correctmotif[1]==motif[1] && motifs!=0){
            telemetry.addLine("second ball is correct");
        }
        else{
          telemetry.addLine("second ball is incorrect");
        }
        if(correctmotif[2]==motif[2] && motifs!=0){
              telemetry.addLine("third ball is correct");
        }
      
        else{
          telemetry.addLine("third ball is incorrect");
        }
        if(correctmotif[2]==motif[2] && motifs!=0 && correctmotif[1]==motif[1] && correctmotif[0]==motif[0]){
          telemetry.addLine("all balls are in correct placement");
          //sleep(50000);
          //go to next april tag and shot
        }
      }

      backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
      currentpositionBL=backleft.getCurrentPosition();
      currentpositionBR=backright.getCurrentPosition();
      currentpositionFR=frontright.getCurrentPosition();
      currentpositionFL=frontleft.getCurrentPosition();
      powerBL=BLA(currentpositionBL,788);//change sign if needed //change value if needed
      powerFL=FLA(currentpositionFL,788);//change sign if needed //change value if needed
      powerFR=FRA(currentpositionFR,788);//change sign if needed //change value if needed
      powerBR=BRA(currentpositionBR,788);//change sign if needed //change value if needed
      frontright.setPower(powerFR);       
      frontleft.setPower(powerFL);         
      backleft.setPower(powerBL);            
      backright.setPower(powerBR);

        

      //servostatusA=0;
      //double pivotdegA=servoA(servostatusA);
      //artifactholder.setPosition(pivotdegA);
      //servostatusB=0;
      //double pivotdegB=servoB(servostatusB);
      //shooterholder.setPosition(pivotdegB);
      //servostatusC=0;
      //double pivotdegC=servoC(servostatusC);
      //belt.setPower(pivotdegC);

      //servostatusE=0;
      //double pivotdegE=servoD(servostatusE);
      //pivotintakeA.setPosition(pivotdegE);
      
      List<AprilTagDetection> myAprilTagDetections;
      AprilTagDetection myAprilTagDetection;
      myAprilTagDetections = myAprilTagProcessor.getDetections();
      telemetry.addData("# AprilTags Detected", JavaUtil.listLength(myAprilTagDetections));
      // Iterate through list and call a function to display info for each recognized AprilTag.
      for (AprilTagDetection myAprilTagDetection_item : myAprilTagDetections) {
          myAprilTagDetection = myAprilTagDetection_item;
        // Display info about the detection.
          telemetry.addLine("");
          if (myAprilTagDetection.metadata != null) {
            telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") " + myAprilTagDetection.metadata.name);
           // Only use tags that don't have Obelisk in them since Obelisk tags don't have valid location data
            test=myAprilTagDetection.id;
            motifs=test;
            
            if (!contains(myAprilTagDetection.metadata.name, "Obelisk")) {
              
              Y=Math.round(myAprilTagDetection.robotPose.getPosition().y*10);
              X=Math.round(myAprilTagDetection.robotPose.getPosition().x*10);
              Z=Math.round(myAprilTagDetection.robotPose.getPosition().z*10);
              Pitch=Math.round(myAprilTagDetection.robotPose.getOrientation().getPitch()*10);
              Roll=Math.round(myAprilTagDetection.robotPose.getOrientation().getRoll()*10);
              Yaw=Math.round(myAprilTagDetection.robotPose.getOrientation().getYaw()*10);              


              bearingA=Math.toDegrees(Math.atan2(X/10, Y/10));
              //telemetry.addLine("XYZ " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().x, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().y, 6, 1) + " " + JavaUtil.formatNumber(myAprilTagDetection.robotPose.getPosition().z, 6, 1) + "  (inch)");
              telemetry.addLine("XYZ " + X/10 + " " + Y/10 + " " + Z/10 + "  (inch)");
              telemetry.addLine("PRY " +  Pitch/10 + " " + Roll/10 + " " + Yaw/10 + " \u03B8 (deg)");
              backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
              frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
              frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
              backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
              currentpositionBL=normalizecurrentangle(orientation.getYaw(AngleUnit.DEGREES));
              currentpositionBR=normalizecurrentangle(orientation.getYaw(AngleUnit.DEGREES));
              currentpositionFR=normalizecurrentangle(orientation.getYaw(AngleUnit.DEGREES));
              currentpositionFL=normalizecurrentangle(orientation.getYaw(AngleUnit.DEGREES));
              powerBL=YawAcontrol(currentpositionBL,normalizecurrentangle((Yaw/-10)));//change sign if needed
              powerFL=YawAcontrol(currentpositionFL,normalizecurrentangle((Yaw/-10)));//change sign if needed
              powerFR=YawAcontrol(currentpositionFR,normalizecurrentangle((Yaw/10)));//change sign if needed
              powerBR=YawAcontrol(currentpositionBR,normalizecurrentangle((Yaw/10)));//change sign if needed
              frontright.setPower(powerFR);       
              frontleft.setPower(powerFL);         
              backleft.setPower(powerBL);            
              backright.setPower(powerBR);
              currentpositionY=intake.getCurrentPosition();
              currentpositionX=X.getCurrentPosition(); 
              deltaX=(X/10)-currentpositionX;
              deltaY=(Y/10)-currentpositionY;
              angletarget=Math.toDegrees(Math.atan2(deltaY,deltaX));
              currentheading=orientation.getYaw(AngleUnit.DEGREES);
              bearingoftag=normalizecurrentangle(bearingA);
              bearingofbot=bearingAngleFound(currentheading,angletarget);
              backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
              frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
              frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
              backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
              powerBL=bearingAcontrol(bearingoftag,bearingofbot);//change sign if needed
              powerFL=bearingAcontrol(bearingoftag,bearingofbot);//change sign if needed
              powerFR=bearingAcontrol(bearingoftag,bearingofbot);//change sign if needed
              powerBR=bearingAcontrol(bearingoftag,bearingofbot);//change sign if needed
              frontright.setPower(powerFR);       
              frontleft.setPower(powerFL);         
              backleft.setPower(powerBL);            
              backright.setPower(powerBR);
              startx=X.getCurrentPosition();
              starty=intake.getCurrentPosition();
              rangeA=Math.sqrt(Math.pow(((X)/10), 2) + Math.pow(((Y)/10), 2));
              rangeB=Math.sqrt(Math.pow(((startx - currentpositionX) * Math.PI * 1.25984) / 2000, 2) + Math.pow(((starty - currentpositionY) * Math.PI * 1.25984) / 2000, 2));
              pointCx=rangeB+offsetX;
              rangeofbot=rangeAcontrol(rangeA,rangeB);
              backleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
              frontleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
              frontright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
              backright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
              powerBL=rangeofbot;//change sign if needed
              powerFL=rangeofbot;//change sign if needed
              powerFR=rangeofbot;//change sign if needed
              powerBR=rangeofbot;
              frontright.setPower(powerFR);       
              frontleft.setPower(powerFL);         
              backleft.setPower(powerBL);            
              backright.setPower(powerBR);
              
            }
        } else {
             telemetry.addLine("==== (ID " + myAprilTagDetection.id + ") Unknown");
             telemetry.addLine("Center " + JavaUtil.formatNumber(myAprilTagDetection.center.x, 6, 0) + "" + JavaUtil.formatNumber(myAprilTagDetection.center.y, 6, 0) + " (pixels)");
            }
  
        }
        telemetry.addLine("");
        telemetry.addLine("key:");
        telemetry.addLine("XYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        
        }
      }
      
      if(correctmotif[0]!=motif[0] && correctmotif[1]==motif[1]){
          servostatusC=-1;
          pivotdegC=servoC(servostatusC);
          belt.setPower(pivotdegC);
          servostatusE=1;
          // might need a delay
          pivotdegE=servoE(servostatusE);
          pivotintakeA.setPosition(pivotdegE);
          servostatusB=1;
          // might need a delay
          pivotdegB=servoB(servostatusB);
          shooterholder.setPosition(pivotdegB);
          double VelocityB=0;//tune this
          double AccelerationB=0;//tune this
          double KSshooterB=0;//tune this
          double KVshooterB=0;//tune this
          double KAshooterB=0;//tune this
          double desiredvelocityB=0;//tune this
          double VelocityA=0;//tune this
          double AccelerationA=0;//tune this
          double KSshooterA=0;//tune this
          double KVshooterA=0;//tune this
          double KAshooterA=0;//tune this
          double desiredvelocityA=0;//tune this
          double powerB=shooterB(((shooterwheelB.getCurrentPosition()/383.6)*Circumference*(96/32))/runtime.seconds(), desiredvelocityB, VelocityB, AccelerationB, KSshooterB, KVshooterB, KAshooterB);
          double powerA=shooterA(((shooterwheelA.getCurrentPosition()/383.6)*Circumference*(96/32))/runtime.seconds(), desiredvelocityA, VelocityA, AccelerationA, KSshooterA, KVshooterA, KAshooterA);
          shooterwheelA.setPower(powerA);
          
          
          
      
        
          
          shooterwheelB.setPower(powerB);
          //might need a delay
          shooterwheelB.setPower(0);  
          shooterwheelA.setPower(0);
      }
      if(correctmotif[1]==motif[1] && correctmotif[0]==motif[0]{
          servostatusD=1;
          // might need a delay
          double pivotdegD=servoD(servostatusD);
          pivotintake.setPosition(pivotdegD);
          servostatusC=1;
          double pivotdegC=servoC(servostatusC);
          
          belt.setPower(pivotdegC);
          servostatusD=0;
          pivotdegD=servoD(servostatusD);
          // might need a delay
          pivotintake.setPosition(pivotdegD);
          double VelocityB=0;//tune this
          double AccelerationB=0;//tune this
          double KSshooterB=0;//tune this
          double KVshooterB=0;//tune this
          double KAshooterB=0;//tune this
          double desiredvelocityB=0;//tune this
          double VelocityA=0;//tune this
          double AccelerationA=0;//tune this
          double KSshooterA=0;//tune this
          double KVshooterA=0;//tune this
          double KAshooterA=0;//tune this
          double desiredvelocityA=0;//tune this
          double powerB=shooterB(((shooterwheelB.getCurrentPosition()/383.6)*Circumference*(96/32))/runtime.seconds(), desiredvelocityB, VelocityB, AccelerationB, KSshooterB, KVshooterB, KAshooterB);
          double powerA=shooterA(((shooterwheelA.getCurrentPosition()/383.6)*Circumference*(96/32))/runtime.seconds(), desiredvelocityA, VelocityA, AccelerationA, KSshooterA, KVshooterA, KAshooterA);
          shooterwheelA.setPower(powerA);
          
          
          
          // might need a delay
        
          
          shooterwheelB.setPower(powerB);
          //might need a delay
          shooterwheelB.setPower(0);  
          shooterwheelA.setPower(0);
      }
      if(correctmotif[1]!=motif[1]){
          servostatusD=1;
          // might need a delay
          double pivotdegD=servoD(servostatusD);
          pivotintake.setPosition(pivotdegD);
          servostatusC=1;
          double pivotdegC=servoC(servostatusC);
          
          belt.setPower(pivotdegC);
          servostatusD=0;
          pivotdegD=servoD(servostatusD);
          // might need a delay
          pivotintake.setPosition(pivotdegD);
          double VelocityB=0;//tune this
          double AccelerationB=0;//tune this
          double KSshooterB=0;//tune this
          double KVshooterB=0;//tune this
          double KAshooterB=0;//tune this
          double desiredvelocityB=0;//tune this
          double VelocityA=0;//tune this
          double AccelerationA=0;//tune this
          double KSshooterA=0;//tune this
          double KVshooterA=0;//tune this
          double KAshooterA=0;//tune this
          double desiredvelocityA=0;//tune this
          double powerB=shooterB(((shooterwheelB.getCurrentPosition()/383.6)*Circumference*(96/32))/runtime.seconds(), desiredvelocityB, VelocityB, AccelerationB, KSshooterB, KVshooterB, KAshooterB);
          double powerA=shooterA(((shooterwheelA.getCurrentPosition()/383.6)*Circumference*(96/32))/runtime.seconds(), desiredvelocityA, VelocityA, AccelerationA, KSshooterA, KVshooterA, KAshooterA);
          shooterwheelB.setPower(powerB);
          shooterwheelA.setPower(powerA);
          // might need a delay
          shooterwheelA.setPower(0);
          servostatusC=-1;
          pivotdegC=servoC(servostatusC);
          belt.setPower(pivotdegC);
          servostatusE=1;
          // might need a delay
          pivotdegE=servoE(servostatusE);
          pivotintakeA.setPosition(pivotdegE);
          servostatusB=1;
          // might need a delay
          pivotdegB=servoB(servostatusB);
          shooterholder.setPosition(pivotdegB);
          shooterwheelA.setPower(0);
      }  
      // Get a list of AprilTag detections.
    
      //if(correctmotif[2]!=motif[2] && motifs!=0 && correctmotif[1]!=motif[1] && correctmotif[0]!=motif[0]){
      //  telemetry.addLine("all balls are in incorrect placement");
        //sleep(50000);
        //shot 
      //}

      
     // telemetry.addData("Exposure", myExposure + "  (" + minExposure + " - " + maxExposure + ")");
      //telemetry.addData("Gain", myGain + "  (" + minGain + " - " + maxGain + ")");
      telemetry.update();
      //if (gamepad1.dpad_down) {
        // Temporarily stop the streaming session. This can save CPU
        // resources, with the ability to resume quickly when needed.
        //myVisionPortal.stopStreaming();
      //} else if (gamepad1.dpad_up) {
        // Resume the streaming session if previously stopped.
        //myVisionPortal.resumeStreaming();
      //}
      // Share the CPU.
      //sleep(20);
    }
  }
  
  /**
   * Initialize AprilTag Detection.
   */
  private void initAprilTag() {
    AprilTagProcessor.Builder myAprilTagProcessorBuilder;
    VisionPortal.Builder myVisionPortalBuilder;

    // First, create an AprilTagProcessor.Builder.
    myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();
    myAprilTagProcessorBuilder.setCameraPose(cameraPosition, cameraOrientation);
    // Create an AprilTagProcessor by calling build.
    myAprilTagProcessor = myAprilTagProcessorBuilder.build();
    // Next, create a VisionPortal.Builder and set attributes related to the camera.
    myVisionPortalBuilder = new VisionPortal.Builder();
    if (USE_WEBCAM) {
      // Use a webcam.
      myVisionPortalBuilder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
    } else {
      // Use the device's back camera.
      myVisionPortalBuilder.setCamera(BuiltinCameraDirection.BACK);
    }
    // Add myAprilTagProcessor to the VisionPortal.Builder.
    myVisionPortalBuilder.addProcessor(myAprilTagProcessor);
    // Create a VisionPortal by calling build.
    myVisionPortal = myVisionPortalBuilder.build();
  }

  /**
   * Display info (using telemetry) for a recognized AprilTag.
   */

  /**
   * returns if the containText is inside of the stringToSearch
   */
  private boolean contains(String stringToSearch, String containText) {
    if (stringToSearch.indexOf(containText) + 1 == 0) {
      return false;
    }
    return true;
  }
  private void getCameraSetting() {
    // Wait for the camera to be open.
    waitForCamera();
    // Get camera control values unless we are stopping.
    if (!isStopRequested()) {
      // Get the ExposureControl object, to allow adjusting the camera's exposure.
      myExposureControl = myVisionPortal.getCameraControl(ExposureControl.class);
      minExposure = 10;
      maxExposure = 30;
      // Get the GainControl object, to allow adjusting the camera's gain.
      myGainControl = myVisionPortal.getCameraControl(GainControl.class);
      minGain = 0;
      maxGain = 100;
    }
  }

  /**
   * Wait for the camera to be open.
   */
  private void waitForCamera() {
    if (!myVisionPortal.getCameraState().equals(VisionPortal.CameraState.STREAMING)) {
      telemetry.addData("Camera", "Waiting");
      telemetry.update();
      while (!isStopRequested() && !myVisionPortal.getCameraState().equals(VisionPortal.CameraState.STREAMING)) {
        sleep(20);
      }
      telemetry.addData("Camera", "Ready");
      telemetry.update();
    }
  }
}
}
}}}
