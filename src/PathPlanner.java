

import java.util.ArrayList;
import java.util.Arrays;

public class PathPlanner {
    public double targetX, targetY;
    private boolean purePursuitIsActive = false;
    private double power;
    public final double robotWidth = 1; //TODO: set correct constant
    public final double distEpsilon = 0.1; //TODO: set correct constant
    public double r_cent = 0;
    public double radius_nonabs = 0;
    public boolean left_cent = false;
    public double[] originCirc;
    public double[] midpoint;
    

    private ArrayList<Segment> pathSegments = new ArrayList<>();
    private Segment currentSegment;
    public double lookAheadDistance;

    private TrackerWheels tracker;


    PathPlanner(double power, double[][] points, TrackerWheels tracker,  double lookAheadDistance) {
        this.power = power;
        this.tracker = tracker;
        this.originCirc = new double[] {0,0};
        this.midpoint = new double[] {0,0};
        double[] coords = tracker.getCoordinates();
        pathSegments.add(new Segment(new double[]{coords[0], coords[1]}, points[0]));
        for (int i = 0; i < points.length - 1; i++) {
            pathSegments.add(new Segment(points[i], points[i + 1]));
        }
        System.out.println(pathSegments.size());
        this.lookAheadDistance = lookAheadDistance;
        runPath();
    }

    //PURE PURSUIT
    public void setTargetPoint(double tx, double ty) {
        targetX = tx;
        targetY = ty;
        purePursuitIsActive = true;
    }

    private void updatePurePursuit() {
        double[] coords = tracker.getCoordinates();
        double x = coords[0];
        double y = coords[1];
        double theta = coords[2];
        double distToTarget = Math.hypot(targetX - x, targetY - y); // distance to target point
        if (distToTarget < distEpsilon) { // we reached the target
            purePursuitIsActive = false;
            return;
        }
        // cross product of heading vector and vector to the target point
        double crossProduct = Math.cos(Math.toRadians(theta)) * (targetY - y) - (targetX - x) * Math.sin(Math.toRadians(theta));
     //   System.out.printf("Cross Value: %s ",crossProduct);
        if (crossProduct == 0) { // if point is straight ahead
            System.out.println("straight ahead");
            return;
        }
        
        double gamma = (Math.atan2(-(targetY - y), (targetX - x)) + 2 * Math.PI) % (2 * Math.PI); // angle between x-axis and vector to the target point
        //beta - angle between vector to the target point and perpendicular to the heading vector
        double beta = (Math.toRadians(theta) + 2 * Math.PI - gamma + 2 * Math.PI) % (2 * Math.PI);
        double r = distToTarget * Math.sin(Math.PI - 2 * beta) / Math.sin(beta); // radius of the turn
        //r_cent = r;
        /** Neel Computations **/
        double normalized_x = (targetX-x)/2.0;
        double normalized_y = (targetY-y)/2.0;
        
        
        double heading = Math.toRadians(theta);
        
        midpoint[0] = x+ normalized_x;
        midpoint[1] = y+normalized_y;
        
        double scaleOfOrthogonal = Math.pow(normalized_y, 2)+ Math.pow(normalized_x, 2);
        
        scaleOfOrthogonal /= normalized_y*Math.cos(heading) -normalized_x* Math.sin(heading);
        
        double deltax = -scaleOfOrthogonal*Math.sin(heading);
        double deltay = scaleOfOrthogonal*Math.cos(heading);
        
        originCirc[0] = x + deltax;
        originCirc[1] = y + deltay;
    //dda    System.out.println("\n"+deltax + " "+deltay);
        r_cent = Math.abs(scaleOfOrthogonal);
        radius_nonabs = scaleOfOrthogonal;
       
        
        double powerRatio = r / (r + robotWidth); // ratio for inner and outer motor powers
        
        if (crossProduct > 0) { // we are making left turn
        	// increase theta by 90, add r cos theta r sin theta
      
        	//System.out.println(Math.cos(-Math.toRadians(theta-90)) + " "+Math.sin(-Math.toRadians(theta-90)));
        	//System.out.println("\n"+Arrays.toString(originCirc) + " radius of "+r);
        	left_cent = true;
        	//System.out.println("Left Turn "+powerRatio);
        } else {
        	//System.out.println("\n"+Arrays.toString(originCirc) + " radius of "+r);
        	left_cent = false;
        	//System.out.println("Right Turn "+powerRatio);
        }
        
    }

    //PATH LOGIC
    private double[] segmentCircleIntersection(Segment segment) {
        double[] coords = tracker.getCoordinates();
        double x = coords[0];
        double y = coords[1];
        double a = Math.pow(segment.x2 - segment.x1, 2) + Math.pow(segment.y2 - segment.y1, 2);
        double b = 2 * (segment.x2 - segment.x1) * (segment.x1 - x) + 2 * (segment.y2 - segment.y1) * (segment.y1 - y);
        double c = Math.pow(segment.x1 - x, 2) + Math.pow(segment.y1 - y, 2) - lookAheadDistance * lookAheadDistance;
        if (b * b - 4 * a * c >= 0) {
            double t = Math.max(2 * c / (-b + Math.sqrt(b * b - 4 * a * c)), 2 * c / (-b - Math.sqrt(b * b - 4 * a * c)));
            if (t >= 0 && t <= 1) {
                return new double[]{segment.x1 + (segment.x2 - segment.x1) * t,
                        segment.y1 + (segment.y2 - segment.y1) * t};
            } else {
                return new double[]{Double.NaN, Double.NaN};
            }
        }
        return new double[]{Double.NaN, Double.NaN};
    }
    public boolean noIntersections(ArrayList<Segment> s) {
    	for(Segment k : s) {
    		if (!Double.isNaN(segmentCircleIntersection(k)[0])) {return false;}
    	}
    	return true;
    }
    public void runPath() {
    		if (pathSegments.size() == 0) {return;}
    		if (!purePursuitIsActive) { // if we reached current target
    			currentSegment = pathSegments.get(0); // set next segment as current
            }
    		//if (Double.isNaN(currentSegment))
            //while we can reach next segment
    		//System.out.println(pathSegments.isEmpty()+ "     "+segmentCircleIntersection(pathSegments.get(0))[0]);
    		if(noIntersections(pathSegments)) {
                setTargetPoint(currentSegment.x2, currentSegment.y2); // go directly to the next point
    			updatePurePursuit();
    			return;
    		}
            while (!pathSegments.isEmpty() && Double.isNaN(segmentCircleIntersection(pathSegments.get(0))[0])) {
            	

            	pathSegments.remove(0);
            	if (pathSegments.size() == 0) {
            		return;
            	}
                currentSegment = pathSegments.get(0); // set next segment as current
            }
            
            double[] coords = segmentCircleIntersection(currentSegment);
            //System.out.printf("intersecting on a circle: result, %s %s \n", coords[0], coords[1]);
            //System.out.printf("isNan: %s \n",Double.isNaN(coords[0]));
            //System.out.printf("isSegEmpty: %s \n",!pathSegments.isEmpty());
            if (Double.isNaN(coords[0])) { // If no point found
            	//System.out.println("went to edge \n");
                setTargetPoint(currentSegment.x2, currentSegment.y2); // go directly to the next point
            } else {
            	//System.out.printf("went to: %s, %s\n ",coords[0],coords[1]);
                setTargetPoint(coords[0], coords[1]);
            }
            updatePurePursuit();
    }
       
    
}

class TrackerWheels{
	public double[] pos;
	public TrackerWheels(double x, double y, double theta) {
		pos = new double[]{x,y,theta};
	}
	public double[] getCoordinates() {
		return pos;
	}
	
}
class Segment {
    double x1, y1, x2, y2;

    Segment(double[] p1, double[] p2) {
        this.x1 = p1[0];
        this.y1 = p1[1];
        this.x2 = p2[0];
        this.y2 = p2[1];
    }
}