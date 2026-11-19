import java.nio.file.*;
import java.io.*;
import java.util.*;

/** A finite cylindrical rigid body in registered trough coordinates, SI units. */
public class SpatialContact {
    static final double R=.05715, INNER=.0508, HALF=.301625/2, MASS=.816466266, G=9.80665;
    static final double AXIAL=MASS*(R*R+INNER*INNER)/2, CROSS=MASS*(3*(R*R+INNER*INNER)+4*HALF*HALF)/12;
    static final double STIFF=60000, DAMP=1.2*Math.sqrt(STIFF*MASS);
    record Line(double a,double b,double c){}
    record Hole(double x,double y,double r){}
    static class Panel {
        List<Line> edges=new ArrayList<>(); List<Hole> holes=new ArrayList<>();
        boolean supports(double x,double y) {
            for(Line e:edges) if(e.a*x+e.b*y+e.c<.005)return false;
            for(Hole h:holes){double dx=x-h.x,dy=y-h.y;if(dx*dx+dy*dy<=h.r*h.r)return false;}
            return true;
        }
    }
    static final Panel[] panels={new Panel(),new Panel()};
    static double slope,intercept,lipDepth,lipTop,scale,floorX,floorZ;
    static double[] rack;
    double x,y,z,vx,vy,vz,ax,ay,az,lx,ly,lz,wx,wy,wz;
    double fx,fy,fz,tx,ty,tz,compression,baseline,energyGain,spring,minFront=1,minRiser=1,dwell;
    double floorLo,floorHi,lipLo,lipHi,firstFloor=-1,firstLip=-1,mu;
    final double[] input;
    final String label;
    SpatialContact(String[] row) {
        label=row[0];input=new double[row.length-1];
        for(int i=0;i<input.length;i++)input[i]=Double.parseDouble(row[i+1]);
        x=input[0];y=input[1];z=input[2];vx=input[3];vy=input[4];vz=input[5];
        ax=input[6];ay=input[7];az=input[8]; wx=input[9];wy=input[10];wz=input[11];
        double l=(AXIAL-CROSS)*(ax*wx+ay*wy+az*wz);
        lx=CROSS*wx+ax*l;ly=CROSS*wy+ay*l;lz=CROSS*wz+az*l;
        mu=input[19];baseline=energy();
    }
    double energy(){return .5*MASS*(vx*vx+vy*vy+vz*vz)+.5*(lx*wx+ly*wy+lz*wz)+MASS*G*z+spring;}
    void contact(double s,double weight,double nx,double nz,double gap,double h,boolean floor){
        double along=nx*ax+nz*az;
        double radial=-R/h;
        double rx=s*ax+radial*(nx-along*ax), ry=s*ay-radial*along*ay, rz=s*az+radial*(nz-along*az);
        double px=x+rx,py=y+ry;
        if(floor && !panels[0].supports(px*scale,py) && !panels[1].supports(px*scale,py))return;
        // The lip extends along the near edge of the two panels; omit the center seam.
        if(!floor && (Math.abs(py)<.008 || Math.abs(py)>.464))return;
        double ux=vx+wy*rz-wz*ry,uy=vy+wz*rx-wx*rz,uz=vz+wx*ry-wy*rx;
        double vn=ux*nx+uz*nz;
        double normal=weight*Math.max(0,-STIFF*gap-DAMP*vn);
        spring+=.5*weight*STIFF*gap*gap;
        if(normal==0)return;
        if(floor){floorLo=Math.min(floorLo,s);floorHi=Math.max(floorHi,s);}
        else {lipLo=Math.min(lipLo,s);lipHi=Math.max(lipHi,s);}
        compression=Math.max(compression,-gap);
        ux-=vn*nx;uz-=vn*nz;
        double speed=Math.sqrt(ux*ux+uy*uy+uz*uz);
        double friction=speed>1e-14?-mu*normal*Math.tanh(speed/.01)/speed:0;
        double a=normal*nx+friction*ux,b=friction*uy,c=normal*nz+friction*uz;
        fx+=a;fy+=b;fz+=c;
        tx+=ry*c-rz*b;ty+=rz*a-rx*c;tz+=rx*b-ry*a;
    }
    void contacts(int stations){
        fx=fy=tx=ty=tz=spring=0;fz=-MASS*G;
        floorLo=lipLo=1;floorHi=lipHi=-1;
        double proj=ax*floorX+az*floorZ;
        double hr=Math.sqrt(Math.max(0,1-proj*proj));
        for(int j=0;j<stations;j++){
            double s=HALF*(2.*j/(stations-1)-1);
            double weight=(j==0 || j==stations-1?.5:1)/(stations-1);
            double px=x+s*ax,pz=z+s*az;
            double gap=px*floorX+(pz-intercept)*floorZ-R*hr;
            if(gap<=0)contact(s,weight,floorX,floorZ,gap,hr,true);
            double dx=px-Math.min(lipDepth,Math.max(0,px)), dz=pz-Math.min(lipTop,Math.max(.003175,pz));
            double d=Math.hypot(dx,dz);
            if(d<R && d>1.e-12) {
                double nx=dx/d,nz=dz/d;
                double a=nx*ax+nz*az,h=Math.sqrt(Math.max(0,1-a*a));
                if(d<R*h)contact(s,weight,nx,nz,d-R*h,h,false);
            }
        }
    }
    void step(double dt){
        vx+=dt*fx/MASS;vy+=dt*fy/MASS;vz+=dt*fz/MASS;
        x+=dt*vx;y+=dt*vy;z+=dt*vz;
        lx+=dt*tx;ly+=dt*ty;lz+=dt*tz;
        double l=(ax*lx+ay*ly+az*lz)*(1/AXIAL-1/CROSS);
        wx=lx/CROSS+ax*l;wy=ly/CROSS+ay*l;wz=lz/CROSS+az*l;
        double a=ax,b=ay,c=az;
        ax+=dt*(wy*c-wz*b);ay+=dt*(wz*a-wx*c);az+=dt*(wx*b-wy*a);
        double norm=Math.sqrt(ax*ax+ay*ay+az*az);ax/=norm;ay/=norm;az/=norm;
    }
    static double withdrawal(double t){
        if(t<=0)return 0;
        double index=Math.min(rack.length-1,t*1000);
        int i=(int)index;
        return rack[i]+(index-i)*(rack[Math.min(i+1,rack.length-1)]-rack[i]);
    }
    void run(PrintWriter report,PrintWriter trace){
        double cx=input[12],cy=input[13],heading=input[14],start=input[15],end=input[16],dt=input[17];
        int stations=(int)input[18],steps=(int)Math.ceil(end/dt);
        double co=Math.cos(heading),si=Math.sin(heading),t=0;
        String result="elapsed";
        for(int i=0;i<=steps;i++){
            t=i*dt;contacts(stations);
            if(floorLo<floorHi && firstFloor<0)firstFloor=t;
            if(lipLo<lipHi && firstLip<0)firstLip=t;
            double withdrawn=withdrawal(t-start);
            double robotX=(x-cx)*co+(y-cy)*si, axisX=ax*co+ay*si;
            // Finite cylinder has this exact support extent along the chassis fore-aft axis.
            double back=robotX-HALF*Math.abs(axisX)-R*Math.sqrt(Math.max(0,1-axisX*axisX));
            double front=Math.max(.425,.535-withdrawn)+.001;
            double gap=back-front;
            // Prior to first field contact the tube is deliberately tangent to the shelf.
            if(firstFloor>=0){
                minFront=Math.min(minFront,gap);
                // A negative longitudinal bound is inconclusive for whole-robot collision.
                if(gap<=0){result="front-separation-unproven";break;}
            }
            // A separating X-plane suffices for the two registered inner risers.
            double ahead=x+HALF*Math.abs(ax)+R*Math.sqrt(Math.max(0,1-ax*ax));
            minRiser=Math.min(minRiser,.2478366565710712-ahead);
            if(minRiser<=0){result="inner-riser-separation-unproven";break;}
            energyGain=Math.max(energyGain,energy()-baseline);
            if(floorHi-floorLo>=.18 && lipHi-lipLo>=.18 &&
               vx*vx+vy*vy+vz*vz<.0001 && wx*wx+wy*wy+wz*wz<.04 && gap>.001)dwell+=dt;
            else dwell=0;
            if(i%Math.max(1,(int)Math.round(.05/dt))==0 && t<=3)
                trace.printf(Locale.ROOT,"%s,%.5f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f%n",label,t,x,y,z,ax,ay,az,gap);
            if(i<steps)step(dt);
        }
        if(result.equals("elapsed") && dwell>=.25)result="supported";
        report.printf(Locale.ROOT,"%s,%s,%.6f,%.6f,%.6f,%.6f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f%n",
            label,result,t,firstFloor,firstLip,dwell,x,y,z,ax,ay,az,minFront,minRiser,compression,energyGain);
    }
    public static void main(String[] args)throws Exception{
        for(String s:Files.readAllLines(Path.of(args[0]))){
            String[] r=s.split(",");double[] d=Arrays.stream(r).skip(2).mapToDouble(Double::parseDouble).toArray();
            if(r[0].equals("profile")){slope=d[0];intercept=d[1];lipDepth=d[2];lipTop=d[3];scale=Math.hypot(1,slope);floorX=-slope/scale;floorZ=1/scale;}
            if(r[0].equals("edge"))panels[Integer.parseInt(r[1])].edges.add(new Line(d[0],d[1],d[2]));
            if(r[0].equals("hole"))panels[Integer.parseInt(r[1])].holes.add(new Hole(d[0],d[1],d[2]));
        }
        rack=Files.lines(Path.of(args[1])).skip(1).map(s->s.split(",")[1]).mapToDouble(Double::parseDouble).toArray();
        try(PrintWriter report=new PrintWriter(args[3]);PrintWriter trace=new PrintWriter(args[4])){
            report.println("case,outcome,elapsed_s,first_floor_s,first_lip_s,final_dwell_s,x_m,y_m,z_m,axis_x,axis_y,axis_z,min_front_m,min_riser_m,max_compression_m,energy_gain_J");
            trace.println("case,t_s,x_m,y_m,z_m,axis_x,axis_y,axis_z,front_gap_m");
            for(String s:Files.readAllLines(Path.of(args[2])))new SpatialContact(s.split(",")).run(report,trace);
        }
    }
}


