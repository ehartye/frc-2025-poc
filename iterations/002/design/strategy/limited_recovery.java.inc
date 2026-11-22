    double angle,angularRate,integral,command,nextSample,gearEfficiency,stowDuration,stowedAt=-1;
    double peakPhase,peakV,peakWatts,peakRate;
    static PrintWriter motorRecord;
    void advancePan(double wall,double dt) {
        double w=Math.max(0,Math.min(1,(wall-closeAt)/1.6));
        double dump=Math.toRadians(-2)-pitch, span=Math.toRadians(70)-dump;
        double target=dump+span*w*w*(3-2*w);
        double velocity=w>0 && w<1 ? span*6*w*(1-w)/1.6 : 0;
        double accel=w>0 && w<1 ? span*(6-12*w)/(1.6*1.6) : 0;
        double rx=Math.cos(angle)*.119719761676108-Math.sin(angle)*.0444888115529926;
        double rz=Math.sin(angle)*.119719761676108+Math.cos(angle)*.0444888115529926;
        double gravity=3.3452483934046366*9.80665*(rx*Math.cos(pitch)-rz*Math.sin(pitch));
        double inertia=.11503479927879083,ratio=(68./13)*(76./21)*(84./29);
        double torquePerAmp=ratio*gearEfficiency*.0097, limit=(30-1.4)*torquePerAmp;
        if(wall>=nextSample-1e-9){
            double delta=target-angle;
            double requested=-inertia*accel-65*delta-2.5*(velocity-angularRate)-integral-gravity;
            command=Math.max(-limit,Math.min(limit,requested));
            if(Math.abs(requested)<limit || Math.signum(requested)==Math.signum(delta))
                integral=Math.max(-2,Math.min(2,integral+15*delta*.02));
            nextSample=wall+.020;
        }
        double ke=(12-.12*1.4)/(11000*Math.PI/30);
        double available=Math.max(0,Math.min(30,(9-ke*Math.abs(angularRate)*ratio)/.136));
        double maximum=Math.max(0,available-1.4)*torquePerAmp;
        double tq=Math.max(-maximum,Math.min(maximum,command));
        double current=1.4+Math.abs(tq)/torquePerAmp;
        double volt=current*.136+ke*Math.abs(angularRate)*ratio;
        peakPhase=Math.max(peakPhase,current);peakV=Math.max(peakV,volt);
        peakWatts=Math.max(peakWatts,current*volt/.90+.7);peakRate=Math.max(peakRate,Math.abs(angularRate));
        if(Double.isFinite(closeAt)&&wall>=closeAt+1.6 && Math.abs(angle-Math.toRadians(70))<=Math.toRadians(1)
                && Math.abs(angularRate)<=.03)stowDuration+=dt; else stowDuration=0;
        if(stowedAt<0 && stowDuration>=.10)stowedAt=wall;
        angularRate+=-(tq+gravity)*dt/inertia;
        angle+=angularRate*dt;
    }

