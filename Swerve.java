import edu.wpi.first.wpilibj.motorcontrol.MotorController;

Robot.steeringratio // the proportion of the steering motor speed needed to keep the wheel not rolling
Robot.turningmultiplier // the fraction of motor max speed which is the maximum turning speed of the wheel
Robot.speedmultiplier // the fraction of motor max speed which is the maximum drive speed of the wheel


/**
 * Calculate an angle from an x and y value.
 * <p>
 * Positive y is 0, and positive x is pi/2.
 * @return The angle in radians.
 */
double Util.getAngle(double x,double y){
  if(abs(x)>abs(y)){
    return Math.PI/2-atan(y/x)-(x<0?Math.PI:0);
  }
  return atan(x/y)+(y<0?Math.PI:0);
} // +y is 0 +x is 1/2pi

/**
 * Calculate the angle from b to a.
 * <p>
 * For the wheels.
 * Is mod pi.
 * @return The angle in radians.
 */
double Util.angleDiff(double a,double b){
  double diff=a-b;
  if(diff>Util.HALF_PI){
    diff-=Math.PI;
  }else if(diff<-Util.HALF_PI){
    diff+=Math.PI;
  }
  return diff;
} // the clockwise angle from b to a mod pi

/**
 * @author      Robert Vail <robert@robertvail.info>
 */

class SwerveController{
  /**
   * The target sideways velocity. Positive values mean move right.
   */
  private double xv;
  
  /**
   * The target forwards velocity.
   */
  private double yv;
  
  /**
   * The target turning speed.
   */
  private double spin;
  
  /**
   * The maximum wheel rotational speed per update.
   */
  private final double maxwrv;
  
  /**
   * The wheels in the swerve drive.
   */
  private Wheel[] wheels;

  /**
   * Change the robot's target velocities.
   *
   * @param  xv The target rightward velocity. Negative values mean move left.
   * @param  yv The target forward velocity. Negative values mean move backwards.
   * @param  spin The target clockwise turn speed.
   */
  public void control(double xv,double yv,double spin){
    this.xv=xv; // the x velocity, relative to the robot (not the field)
    this.yv=yv; // the x velocity, relative to the robot (not the field)
    this.spin=spin; // the clockwise angular velocity
  };

  /**
   * Move the wheel positions towards the target.
   * <p>
   * Calls {@link #controlWheels(double xv,double yv,double spin)} to move the wheels.
   */
  private void moveWheels(){
    // change velocity linearly because that's easy
    
    int[] currentpos=calculateCurrentControls();
    int xv=currentpos[0],yv=currentpos[1],spin=currentpos[2]; // the approximate current control values
    
    double xa=this.xv-xv; // x acceleration
    double ya=this.yv-yv; // y acceleration
    double ra=this.spin-spin; // rotational acceleration
    
    double maxxa=Double.POSITIVE_INFINITY,maxya=Double.POSITIVE_INFINITY,maxra=Double.POSITIVE_INFINITY;
    
    // The calculations I did to get this formula are in SwerveAlgebra.txt
    double xvn,yvn,sqd; // intermediate calculations
    double invddxv,invddyx,invddspin; // the approximate amount of xv, yv, or spin to change the angle by one unit(radians)
    for(Wheel w:wheels){
      xvn=(xv+yn*spin);
      yvn=(yv-xn*spin);
      sqd=(yvn*yvn+xvn*xvn);
      invddxv=sqd/yvn;
      invddyv=sqd/xvn;
      invddspin=sqd/(yn*yvn+xvn*xn);
      maxxa=min(maxxa,abs(invddxv*maxwra));
      maxya=min(maxya,abs(invddyv*maxwra));
      maxra=min(maxra,abs(invddspin*maxwra));
    }
    
    // scale the accelerations so they stay within the bounds
    if(abs(xa)>maxxa||abs(ya)>maxya||abs(ra)>maxra){
      double scale=min(maxxa/abs(xa),maxya/abs(ya),maxra/abs(ra));
      xa*=scale;
      ya*=scale;
      ra*=scale;
    }
    controlWheels(xv+xa,yv+ya,spin+ra);
  }

  /**
   * Calculate approximate control values (xv,yv,spin)
   * <p>
   * Looks at the wheel angles and speeds and calculates control values that fit the movement.
   * The calculations I did to get this formula are in SwerveAlgebra.txt
   * @return The approximate control values.
   */
  private double[] calculateCurrentControls(){
    double sum_xn=0,sum_yn=0,sum_xvn=0,sum_yvn=0;
    double sum_xnxn=0,sum_ynyn=0;
    double sum_xnyvn=0,sum_ynxvn=0;
    double xn,yn,xvn,yvn;
    double angle;
    int count=wheels.length;
    for(Wheel wheel:wheels){
      xn=wheel.getX();
      yn=wheel.getY();
      angle=wheel.getAngle();
      speed=wheel.getSpeed();
      xvn=sin(angle)*speed;
      yvn=cos(angle)*speed;
      sum_xn+=xn;
      sum_yn+=yn;
      sum_xvn+=xvn;
      sum_yvn+=yvn;
      sum_xnxn+=xn*xn;
      sum_ynyn+=yn*yn;
      sum_xnyvn+=xn*yvn;
      sum_ynxvn+=yn*xvn;
    }
    double pspin=(
      +sum_ynxvn*count
      -sum_xnyvn*count
      -sum_yn*sum_xvn
      +sum_xn*sum_yvn
    )/(
      +sum_xnxn*count
      +sum_ynyn*count
      -sum_yn*sum_yn
      -sum_xn*sum_xn
    );
    double pxv=(+sum_xvn-sum_yn*spin)/count;
    double pyv=(+sum_yvn+sum_xn*spin)/count;
    double[] out={pxv,pyv,pspin};
    return out
  }

  /**
   * Set the wheel positions.
   * <p>
   * The function that actually controls the wheels.
   * @param  xv The target rightward velocity. Negative values mean move left.
   * @param  yv The target forward velocity. Negative values mean move backwards.
   * @param  spin The target clockwise turn speed.
   */
  private void controlWheels(double xv,double yv,double spin){
    // calculations...
    double xvel,yvel;
    double vel,angle;
    for(Wheel wheel:wheels){
      xvel=xv+spin*wheels[i].getY();
      yvel=yv-spin*wheels[i].getX();
      vel=Math.sqrt(xvel*xvel+yvel*yvel);
      angle=Util.getAngle(xvel,yvel);
      wheel.setSpeed(vel);
      wheel.turn(Util.angleDiff(angle,wheel.getAngle()));
    }
  }
  
  class Wheel{
    /**
     * The x position of the wheel relative to the center of the robot. Positive values are towards the right.
     */
    private double x;
    
    /**
     * The y position of the wheel relative to the center of the robot. Positive values are towards the front.
     */
    private double y;
    
    /**
     * The motor that drives the wheel.
     */
    private MotorController drive;
    
    /**
     * The motor that turns the wheel.
     */
    private MotorController steering;
    
    /**
     * Make a new Wheel
     * @param  x The x position of the wheel relative to the center of the robot. Positive values are towards the right.
     * @param  y The y position of the wheel relative to the center of the robot. Positive values are towards the front.
     * @param  drive The motor that drives the wheel.
     * @param  steering The motor that turns the wheel.
     */
    Wheel(double x,double y,MotorController drive,MotorController steering){
      this.x=x;
      this.y=y;
      this.drive=drive;
      this.steering=steering;
    }
    
    /**
     * Get the x position of this wheel.
     * @return The x position of this wheel.
     */
    public double getX(int index){return x;}
    
    /**
     * Get the y position of this wheel.
     * @return The y position of this wheel.
     */
    public double getY(int index){return y;}
    
    /**
     * Get the angle of this wheel relative to the front of the robot in radians.
     * <p>
     * Can be mod 2pi.
     * @return The angle of this wheel relative to the front of the robot.
     */
    public double getAngle();
    
    /**
     * Get the speed this wheel is turning.
     * @return The speed this wheel is turning.
     */
    public double getSpeed();
    
    /**
     * Set the turning speed.
     */
    public void turn(double speed){
      double oldspeed=steering.get();
      speed*=Robot.turningmultiplier;
      steering.set(speed);
      drive.set(drive.get()-(speed-oldspeed)*Robot.steeringratio);
    }
    
    /**
     * Set the drive speed.
     */
    public void setSpeed(double speed){
      drive.set(speed*Robot.speedmultiplier-steering.get()*Robot.steeringratio);
    }
  }
}



