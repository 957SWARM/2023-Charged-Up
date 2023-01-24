package frc.robot;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
// Network Table Setup
NetworkTableInstance inst = NetworkTableInstance.getDefault();
NetworkTable table = inst.getTable("limelight");
DoubleSubscriber vSub;
DoubleSubscriber xSub;
DoubleSubscriber ySub;
DoubleSubscriber aSub;
DoubleSubscriber sSub;
DoubleSubscriber lSub;
DoubleSubscriber shortSub;
DoubleSubscriber longSub;
DoubleSubscriber horSub;
DoubleSubscriber idSub;
DoubleSubscriber classSub;
DoubleSubscriber vertSub;
DoubleSubscriber jsonSub;
DoubleSubscriber getpipeSub;
double [] campose = table.getEntry("campose").getDoubleArray(new double[]{});
double [] botpose = table.getEntry("botpose").getDoubleArray(new double[]{});


// Whether or not a target is seen
  public double getTv(){
    vSub = table.getDoubleTopic("tv").subscribe(0);
    double v = vSub.get();
    return v;
  }

// Angle off from target horizontally
  public double getTx(){
    xSub = table.getDoubleTopic("tx").subscribe(0);
    double x = xSub.get();
    return x;
  }

  // Angle off from target vertically
  public double getTy(){
    ySub = table.getDoubleTopic("ty").subscribe(0);
    double y = ySub.get();
    return y;
  }

  public double getTa(){
    aSub = table.getDoubleTopic("ta").subscribe(0);
    double a = aSub.get();
    return a;
  }

   public double getTs(){
    sSub = table.getDoubleTopic("ts").subscribe(0);
    double s = sSub.get();
    return s;
   }

  public double getTl(){
    lSub = table.getDoubleTopic("tl").subscribe(0);
    double l = lSub.get();
    return l;
  }

  public double getTshort(){
    shortSub = table.getDoubleTopic("tshort").subscribe(0);
    double tshort = shortSub.get();
    return tshort;
  }
  
  public double getTlong(){
    longSub = table.getDoubleTopic("tlong").subscribe(0);
    double tlong = longSub.get();
    return tlong;
  }
    
  public double getThor(){
    horSub = table.getDoubleTopic("thor").subscribe(0);
    double thor = horSub.get();
    return thor;
  }
  
  public double getTvert(){
    vertSub = table.getDoubleTopic("tvert").subscribe(0);
    double tvert = vertSub.get();
    return tvert;
  }

  public double getPipe(){
    getpipeSub = table.getDoubleTopic("getpipe").subscribe(0);
    double getpipe = getpipeSub.get();
    return getpipe;
  }
  
    public double getjson(){
    jsonSub = table.getDoubleTopic("tjson").subscribe(0);
    double json = jsonSub.get();
    return json;
  }

    public double getTclass(){
    classSub = table.getDoubleTopic("tclass").subscribe(0);
    double tclass = classSub.get();
    return tclass;
  }
    
  public double getATID(){
    idSub = table.getDoubleTopic("tid").subscribe(0);
    double id = idSub.get();
    return id;
  }

  public double getDistance(){

  campose = table.getEntry("campose").getDoubleArray(new double[]{0,0,0,0,0,0});
  // Secret Krabby Patty Distance Formula (SKPDF)
  double distance = ( (campose[2] * 39.37 - 46.85) / 1.3 ) + 34.5;
  // Secret Krabby Patty Distance Formula 2 (SKPDF2)
  double campose2 = (campose[2] * 39.37)/-1 + 1.25;
  return campose2;
  }

}
