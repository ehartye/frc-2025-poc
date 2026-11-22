import java.io.*;
import java.nio.file.*;
import java.util.*;

/** Independent post-impact comparison with the rotating pan, station feeder and receiver. SI units. */
final class PanContactStudy extends SpatialContact {
    final double pitch,heave,freeClock,permit,endClock;
    final double cyaw,syaw,cp,sp;
    double closeAt=Double.POSITIVE_INFINITY,dwellAt=-1,minGap=Double.POSITIVE_INFINITY,minAt=-1,largestSpeed=0;
    String closest="";
    final double[] robotP=new double[3],robotA=new double[3],bodyP=new double[3],bodyA=new double[3],worst=new double[6];
    final double[][] feed={{-.340,.795,-.255,.685,.192},{-.255,.685,-.035,.623,.192},{-.035,.623,.075,.592,.082}};
    PanContactStudy(String[] row,String[] clock) {
        super(row);
        pitch=Double.parseDouble(clock[1]);heave=Double.parseDouble(clock[2]);
        freeClock=Double.parseDouble(clock[3]);permit=Double.parseDouble(clock[4]);endClock=Double.parseDouble(clock[5]);
        cyaw=Math.cos(input[14]);syaw=Math.sin(input[14]);cp=Math.cos(pitch);sp=Math.sin(pitch);
    }
    void relative(double x,double y,double z,double[] out){
        double horizontal=cyaw*x+syaw*y;
        out[0]=cp*horizontal+sp*z;out[1]=-syaw*x+cyaw*y;out[2]=-sp*horizontal+cp*z;
    }
    static double span(double a){return HALF*Math.abs(a)+R*Math.sqrt(Math.max(0,1-a*a));}
    // A positive separation along any tested plane certifies the entire
    // cylinder and box disjoint; an inconclusive bound is preserved.
    static double bound(double[] p,double[] a,double x,double y,double z,double rx,double ry,double rz){
        double dx=p[0]-x,dy=p[1]-y,dz=p[2]-z;
        double best=Math.max(Math.abs(dx)-rx-span(a[0]),Math.max(Math.abs(dy)-ry-span(a[1]),Math.abs(dz)-rz-span(a[2])));
        best=Math.max(best,Math.abs(dx*a[0]+dy*a[1]+dz*a[2])-HALF-rx*Math.abs(a[0])-ry*Math.abs(a[1])-rz*Math.abs(a[2]));
        // The common near-tangent case lies diagonally below the end of the
        // pan. A single X or Z plane is inconclusive there. Project the whole
        // finite cylinder and the whole box on three additional normals.
        if(best<.002){
            double lo=-HALF,hi=HALF;
            // Closest point on the finite axis to this oriented box. A
            // projection is sound for every chosen direction, irrespective of
            // minimizer accuracy. This resolves a narrow end-plate corner.
            for(int iter=0;iter<16;iter++){
                double h=(lo+hi)/2;
                double qx=dx+h*a[0],qy=dy+h*a[1],qz=dz+h*a[2];
                double g=(qx-Math.max(-rx,Math.min(rx,qx)))*a[0]+(qy-Math.max(-ry,Math.min(ry,qy)))*a[1]
                        +(qz-Math.max(-rz,Math.min(rz,qz)))*a[2];
                if(g<0)lo=h;else hi=h;
            }
            for(double h:new double[]{-HALF,0,HALF,(lo+hi)/2}){
                double qx=dx+h*a[0],qy=dy+h*a[1],qz=dz+h*a[2];
                double nx=qx-Math.max(-rx,Math.min(rx,qx));
                double ny=qy-Math.max(-ry,Math.min(ry,qy));
                double nz=qz-Math.max(-rz,Math.min(rz,qz));
                double n=Math.sqrt(nx*nx+ny*ny+nz*nz);
                if(n==0)continue;
                nx/=n;ny/=n;nz/=n;
                double projection=span(nx*a[0]+ny*a[1]+nz*a[2]);
                best=Math.max(best,Math.abs(nx*dx+ny*dy+nz*dz)-rx*Math.abs(nx)-ry*Math.abs(ny)-rz*Math.abs(nz)-projection);
            }
        }
        return best;
    }
    void note(String member,double gap,double t,double budget) {
        if(gap-budget<minGap){
            minGap=gap-budget;minAt=t;closest=member;
            for(int j=0;j<3;j++){worst[j]=robotP[j];worst[3+j]=robotA[j];}
        }
    }
    static void rotateFrame(double[] p,double[] a,double x0,double z0,double c,double s,double[] loc,double[] axis){
        double x=p[0]-x0,z=p[2]-z0;
        loc[0]=c*x+s*z;loc[1]=p[1];loc[2]=-s*x+c*z;
        axis[0]=c*a[0]+s*a[2];axis[1]=a[1];axis[2]=-s*a[0]+c*a[2];
    }
    void checkRobot(double t,double budget){
        relative(x-input[12],y-input[13],z-heave,robotP);
        relative(ax,ay,az,robotA);
        double q=Math.max(0,Math.min(1,(freeClock+t-closeAt)/1.6));
        double theta0=Math.toRadians(-2)-pitch,angle=theta0+(Math.toRadians(70)-theta0)*q*q*(3-2*q);
        rotateFrame(robotP,robotA,.205,.580,Math.cos(angle),Math.sin(angle),bodyP,bodyA);
        note("pan_floor",bound(bodyP,bodyA,.125,0,-.001,.215,.161,.001),t,budget);
        for(int side:new int[]{-1,1})
            note("pan_side_"+side,bound(bodyP,bodyA,.125,side*.162,.06,.215,.001,.06),t,budget);
        double length=Math.hypot(.010,.110),c=.010/length,s=-.110/length;
        double[] lipP=new double[3],lipA=new double[3];
        rotateFrame(bodyP,bodyA,-.1,.110,c,s,lipP,lipA);
        for(int side:new int[]{-1,1})
            note("slotted_aft_"+side,bound(lipP,lipA,length/2,side*(.161+.096)/2,-.001,length/2,(.161-.096)/2,.001),t,budget);
        for(int i=0;i<feed.length;i++){
            double[] f=feed[i];double dx=f[2]-f[0],dz=f[3]-f[1],l=Math.hypot(dx,dz);
            rotateFrame(robotP,robotA,f[0],f[1],dx/l,dz/l,bodyP,bodyA);
            note("fixed_feed_"+i,bound(bodyP,bodyA,l/2,0,-.001,l/2,f[4],.001),t,budget);
        }
        // Both stationary inlet cheeks finish before X -0.035. Their swept
        // projection fits into this conservative rectangular prism.
        note("fixed_inlet_cheeks",bound(robotP,robotA,-.1875,0,.767,.1525,.196,.155),t,budget);
    }
    void execute(PrintWriter out){
        double dt=input[17];int stations=(int)input[18];double firstContact=-1,dwell=0;
        boolean retained=true;double lostContactSec=0,latestTime=0;
        final double end=Math.max(input[16], endClock>=0?endClock-freeClock:.0);
        for(int i=0;i<=Math.ceil(end/dt);i++){
            double t=dt*i;latestTime=t;contacts(stations);
            boolean floor=floorHi-floorLo>=.18,lip=lipHi-lipLo>=.18;
            if(firstContact<0&&(floorHi>floorLo||lipHi>lipLo))firstContact=t;
            boolean supported=floor&&lip&&vx*vx+vy*vy+vz*vz<.0001&&wx*wx+wy*wy+wz*wz<.04;
            dwell=supported?dwell+dt:0;
            if(dwell>=.25&&dwellAt<0){
                dwellAt=t;
                if(permit>=0)closeAt=Math.max(permit,freeClock+t+.05);
            }
            if(dwellAt>=0&&!supported){retained=false;lostContactSec+=dt;}
            if(firstContact>=0){
                double nv=Math.sqrt(Math.pow(vx+fx*dt/MASS,2)+Math.pow(vy+fy*dt/MASS,2)+Math.pow(vz+fz*dt/MASS,2));
                double angularBound=Math.sqrt(Math.pow(lx+tx*dt,2)+Math.pow(ly+ty*dt,2)+Math.pow(lz+tz*dt,2))/Math.min(AXIAL,CROSS);
                // step() applies semicimplicit position, followed by normalized
                // angular advance. Add the maximum radius/speed of any pan
                // sheet over the same 50â€“200 us closed integration interval.
                double speed=nv+Math.hypot(HALF,R)*angularBound+.40*(1.5/1.6)*(Math.toRadians(72)+Math.abs(pitch));
                largestSpeed=Math.max(largestSpeed,speed);checkRobot(t,speed*dt+1e-7);
            }
            double ahead=x+span(ax);
            if(ahead>=.2478366565710712)break;
            if(i<(int)Math.ceil(end/dt))step(dt);
        }
        double reset=(Double.isFinite(closeAt)?closeAt+1.6:-1);
        out.printf(Locale.ROOT,"%s,%.6f,%.6f,%s,%.6f,%.6f,%.6f,%.9f,%.6f,%s,%.6f,%.6f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f%n",
            label,firstContact,dwellAt,retained,lostContactSec,reset,endClock,minGap,minAt,closest,largestSpeed,latestTime,
            worst[0],worst[1],worst[2],worst[3],worst[4],worst[5]);
    }
    public static void main(String[] args)throws Exception{
        Path dir=Path.of(args[0]);
        for(String line:Files.readAllLines(dir.resolve("panels.csv"))){
            String[] r=line.split(",");double[] d=Arrays.stream(r).skip(2).mapToDouble(Double::parseDouble).toArray();
            if(r[0].equals("profile")){slope=d[0];intercept=d[1];lipDepth=d[2];lipTop=d[3];scale=Math.hypot(1,slope);floorX=-slope/scale;floorZ=1/scale;}
            if(r[0].equals("edge"))panels[Integer.parseInt(r[1])].edges.add(new Line(d[0],d[1],d[2]));
            if(r[0].equals("hole"))panels[Integer.parseInt(r[1])].holes.add(new Hole(d[0],d[1],d[2]));
        }
        Map<String,String[]> schedules=new HashMap<>();
        for(String line:Files.readAllLines(dir.resolve("assessment-clock.csv"))){String[] r=line.split(",");schedules.put(r[0],r);}
        try(PrintWriter out=new PrintWriter(dir.resolve("post-impact-reset.csv").toFile())){
            out.println("case,first_contact_s,supported_at_s,held_after_support,lost_support_s,reset_wall_s,reset_deadline_wall_s,continuous_lower_gap_m,worst_after_release_s,closest_member,motion_bound_mps,elapsed_s,body_x,body_y,body_z,axis_x,axis_y,axis_z");
            for(String line:Files.readAllLines(dir.resolve("cases.csv"))){String[] r=line.split(",");new PanContactStudy(r,schedules.get(r[0])).execute(out);}
        }
    }
}
