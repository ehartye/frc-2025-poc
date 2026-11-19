package org.agentfrc.lesson;
import edu.wpi.first.math.system.plant.DCMotor;
/** Finite hollow-tube upper-exterior contact, with externally constrained orientation. Not general 3D contact. */
public final class CoralContactPlant {
 public record Config(double mass,double outerRadius,double innerRadius,double length,double rollerRadius,double rollerInertia,double rollerZ,double gear,double floorMu){
  public Config{if(!finite(mass,.49,.82)||!finite(outerRadius,.057,.058)||!finite(innerRadius,.050,.051)||!finite(length,.301,.303)||!finite(rollerRadius,.025,.026)||!finite(rollerInertia,.00005,.001)||!finite(rollerZ,.13,.15)||!finite(gear,1,20)||!finite(floorMu,0,.8))throw new IllegalArgumentException("CORAL hypothesis domain");}
 }
 public record Contact(double fx,double fz,double rollerTorque,double constraintMoment,double potential,double normalLoss,double frictionLoss,double overlap,double normal,double tangent){}
 public record Sample(double x,double z,double vx,double vz,double rollerOmega,double angle,double bus,double batteryAmps,double mechanicalResidual,double normalLoss,double frictionLoss,double rollerNormal,double constraintMoment,double maxOverlap){}
 private final Config c;private final DCMotor motor=DCMotor.getNEO(1);private double[] q=new double[9];private final double initialEnergy;
 public CoralContactPlant(Config c,double x,double z,double vx){this.c=c;q[0]=x;q[1]=z;q[2]=vx;validate(q);initialEnergy=energy(q);}
 static boolean finite(double x,double lo,double hi){return Double.isFinite(x)&&x>=lo&&x<=hi;}
 private static Contact law(double delta,double nx,double nz,double vx,double vz,double wr,double R,double k,double damping,double mu,double leverX,double leverZ){
  if(delta<=0)return new Contact(0,0,0,0,0,0,0,0,0,0);double dd=-(nx*vx+nz*vz),factor=1+damping*dd,elastic=k*delta,fn=elastic*Math.max(0,factor),slip=vx*nz-vz*nx-R*wr,ft=-mu*fn*Math.tanh(slip/.01),fx=fn*nx+ft*nz,fz=fn*nz-ft*nx;
  return new Contact(fx,fz,-R*ft,-(leverZ*fx-leverX*fz),.5*k*delta*delta,factor>=0?k*damping*delta*dd*dd:-elastic*dd,-ft*slip,delta,fn,ft);
 }
 public static Contact upper(Config c,double x,double z,double vx,double vz,double wr){
  if(!finite(x,-.35,.35)||!finite(z,.04,.1)||!finite(vx,-5,5)||!finite(vz,-5,5)||!finite(wr,-200,200))throw new IllegalArgumentException("Contact state domain");
  double top=z+c.outerRadius;if(c.rollerZ<=top)throw new IllegalArgumentException("Unsupported bore/vertical-end/interior feature: roller axis must remain above upper exterior");
  double px=Math.max(x-c.length/2,Math.min(0,x+c.length/2)),pz=top,dx=px,dz=pz-c.rollerZ,distance=Math.hypot(dx,dz);if(distance<.001)throw new IllegalArgumentException("Unsupported deep/interior contact");
  return law(c.rollerRadius-distance,dx/distance,dz/distance,vx,vz,wr,c.rollerRadius,5000,5,.8,px-x,pz-z);
 }
 private Contact[] contacts(double[] a){return new Contact[]{law(c.outerRadius-a[1],0,1,a[2],a[3],0,0,50000,5,c.floorMu,0,-c.outerRadius),upper(c,a[0],a[1],a[2],a[3],a[4])};}
 private void validate(double[] a){for(double v:a)if(!Double.isFinite(v))throw new IllegalArgumentException("Nonfinite state");if(!finite(a[0],-.35,.35)||!finite(a[1],.04,.1)||Math.abs(a[2])>5||Math.abs(a[3])>5||Math.abs(a[4])>200)throw new IllegalArgumentException("Finite fixture face domain");for(var p:contacts(a))if(p.overlap>.008)throw new IllegalArgumentException("Excessive assumed compression");}
 private double energy(double[] a){double e=.5*c.mass*(a[2]*a[2]+a[3]*a[3])+.5*c.rollerInertia*a[4]*a[4]+c.mass*9.80665*a[1];for(var p:contacts(a))e+=p.potential;return e;}
 private SharedSupply.Result supply(double[] a,double duty){try{return SharedSupply.solve(12,.02,new double[]{duty},new double[]{a[4]*c.gear/motor.KvRadPerSecPerVolt},new double[]{motor.rOhms});}catch(IllegalArgumentException e){throw new IllegalArgumentException(e.getMessage()+" x="+a[0]+" vx="+a[2]+" rollerOmega="+a[4]+" duty="+duty,e);}}
 private double[] derivative(double[] a,double duty){validate(a);var s=supply(a,duty);double torque=c.gear*motor.KtNMPerAmp*s.armatureAmps()[0];double[] b={a[2],a[3],0,-9.80665,torque/c.rollerInertia,a[4],torque*a[4],0,0};for(var p:contacts(a)){b[2]+=p.fx/c.mass;b[3]+=p.fz/c.mass;b[4]+=p.rollerTorque/c.rollerInertia;b[7]+=p.normalLoss;b[8]+=p.frictionLoss;}return b;}
 private static double[] stage(double[] a,double[] b,double dt){double[] r=a.clone();for(int i=0;i<r.length;i++)r[i]+=b[i]*dt;return r;}
 public Sample step(double duty,double dt){if(!finite(duty,-.2,.2)||!finite(dt,.00001,.0001))throw new IllegalArgumentException("Integrator domain");var a=derivative(q,duty);var b=derivative(stage(q,a,dt/2),duty);var d=derivative(stage(q,b,dt/2),duty);var e=derivative(stage(q,d,dt),duty);for(int i=0;i<q.length;i++)q[i]+=dt*(a[i]+2*b[i]+2*d[i]+e[i])/6;validate(q);return sample(duty);}
 public Sample sample(double duty){var s=supply(q,duty);double moment=0,overlap=0;for(var p:contacts(q)){moment+=p.constraintMoment;overlap=Math.max(overlap,p.overlap);}return new Sample(q[0],q[1],q[2],q[3],q[4],q[5],s.volts(),s.batteryAmps(),energy(q)+q[7]+q[8]-q[6]-initialEnergy,q[7],q[8],contacts(q)[1].normal,moment,overlap);}
}

