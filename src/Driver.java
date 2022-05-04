import java.util.Random;

public class Driver {
public static void main(String[] args) throws InterruptedException {
	//System.out.println("REMEMBDER EVERYTHING IS COUNTER CLOCKWISE BC ITS STUPID");
	/*
	Scanner sc = new Scanner(System.in);
	double x = sc.nextDouble();
	double y = sc.nextDouble();
	double theta = sc.nextDouble();
	*/
	double x = 100;double y = 100; double theta = 1;
	System.out.printf("Cheading: %s %s %s \n",x,y,theta);
	
	System.out.println("Adding Random noise to data: ");

	Random rand = new Random();
	int center = 500;
	
	/*
	double[][] lattice = new double[50][2];
	
	for(int i = 0; i < lattice.length;i++) {
		//lattice[i] = new double[] {300*Math.cos(Math.toRadians(360 * (i/9.0))) + center,300*Math.sin(Math.toRadians(360 * (i/9.0))) + center };
		//lattice[i] = new double[] {900*((double)i/lattice.length - 0.5) + center,300*Math.sin(Math.toRadians(360 * (i*Math.PI*3/4.0))) + center };
		//lattice[i] = new double[] {600*(Math.random() - 0.5) + center,300*(Math.random() - 0.5) + center };
		//lattice[i] = new double[] {900*((double)i/lattice.length - 0.5) + center,300*Math.sin(Math.toRadians(360 * (i/36.0))) + center };
		
	}
	*/
	
	
	
	double[][] lattice = new double[20][2]; // scurve lattice
	for (int i = 0; i < 5;i++) {
		double xt = 300 + 100*Math.cos(Math.toRadians(90-36*i));
		double yt = 200 - 100*Math.sin(Math.toRadians(90-36*i));
		lattice[i][0] = xt;
		lattice[i][1] = yt;
	}
	for (int i = 0; i < 5;i++) {
		double xt = 300 + 100*Math.cos(Math.toRadians(90+36*i));
		double yt = 400 - 100*Math.sin(Math.toRadians(90+36*i));
		lattice[5 + i][0] = xt;
		lattice[5 + i][1] = yt;
	}
	double xt = 300;
	double yt = 500;
	for(int i = 0;i < 10; i++) {
		switch(i % 4) {
		case 0:
			yt -= 200;
			break;
		case 1:
			xt += 100;
			break;
		case 2:
			yt += 200;
			break;
		case 3:
			xt += 100;
			break;
		}
		lattice[10+i][0] = xt;
		lattice[10+i][1] = yt;
	}
   
	
	//[]
	
	TrackerWheels trackInstance = new TrackerWheels(x, y, theta);
	
	int lookAhead = 50;
	PathPlanner p = new PathPlanner(1,lattice,trackInstance ,lookAhead);
	double[] pos = trackInstance.getCoordinates();
	
	AwtControlDemo  awtControlDemo = new AwtControlDemo();
    awtControlDemo.showCanvasDemo(lattice,pos[0],pos[1],pos[2],p,trackInstance);
    Thread.sleep(1000);
    
    int cc = 0;
    while(true) {    	
    	if (p.radius_nonabs != 0) {
    		double power = clipRange(Math.hypot(trackInstance.pos[0]-p.targetX, trackInstance.pos[1]-p.targetY)/lookAhead,0,1);
    		double dtheta = 200.0/(2.0*Math.PI*p.radius_nonabs) ; 
    		dtheta *= power;
    		System.out.println(Math.hypot(trackInstance.pos[0]-p.targetX, trackInstance.pos[1]-p.targetY)/lookAhead);
    		System.out.println(String.format("%s %s ", trackInstance.pos[0]-p.targetX, trackInstance.pos[1]-p.targetY));
    		//System.out.println("Radius of Circle: "+p.radius_nonabs);
    		//System.out.println("Current Location: "+trackInstance.pos[0]+" "+trackInstance.pos[1]);
    		double gtheta = generateAngle(p.originCirc[0]-trackInstance.pos[0],p.originCirc[1]-trackInstance.pos[1], p.radius_nonabs);
    		//System.out.println("Obtained theta: "+gtheta);
    		double newx,newy=0;
    		newx = p.originCirc[0] + p.radius_nonabs*Math.cos(-Math.toRadians(gtheta + dtheta));
        	newy = p.originCirc[1] - p.radius_nonabs*Math.sin(-Math.toRadians(gtheta + dtheta));
        	
        	
    		trackInstance.pos[0] = newx;
    		trackInstance.pos[1] = newy;
    		
    		trackInstance.pos[2] = gtheta + dtheta + 90 ;
    		
    		awtControlDemo.canv.x = newx;
    		awtControlDemo.canv.y = newy;
    		
    		
    		awtControlDemo.canv.theta = trackInstance.pos[2];
    		
    		if(cc == 5) {
    			p.runPath();
    			cc = 0;
    		}
    		else {
    			cc++;
    		}
    		
    		Thread.sleep(5);
    	}
    	
    }
    
 /**   
    Thread.sleep(5000);
    x = 205; y = 120; theta = 0;
    trackInstance.pos = new double[] {x,y,theta};
    p.runPath();
    awtControlDemo.canv.x = x;
    awtControlDemo.canv.y = y;
    awtControlDemo.canv.theta = theta;
    **/ 
}

private static double clipRange(double d, double i, double j) {
	// TODO Auto-generated method stub
	return Math.max(Math.min(j, d), i);
}

private static double generateAngle(double dx, double dy, double r) {
	double oldr = r;
	if (r < 0) {
		r = -r;
	}
	if (dx > r) {
		dx = r;
	}
	if ( dy > r) {
		dy = r;
	}
	if (dx < -r) {
		dx = -r;
	}
	if (dy < -r) {
		dy = -r;
	}
	//System.out.println(dx + " "+dy + " "+r);

	double yprof = Math.toDegrees(Math.asin(dy/r));
	double xprof = Math.toDegrees(Math.acos(dx/r));
	//System.out.println(yprof + " "+xprof);
	if (yprof < 0) {
		if ( oldr < 0) {
			return 180 + ( 180 - xprof );
		}
		else {
			return (180-xprof);
		}
	}
	if(oldr < 0) {
		return xprof;
	}
	else {
		return (xprof+180 )% 360;
	}
}
}
