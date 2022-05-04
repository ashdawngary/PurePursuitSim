import java.awt.Color;
import java.awt.Frame;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GridLayout;
import java.awt.Label;
import java.awt.Panel;
import java.awt.Shape;
import java.awt.event.KeyEvent;
import java.awt.event.KeyListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.geom.Ellipse2D;
import java.util.LinkedList;

import javax.swing.JPanel;

public class AwtControlDemo {

   public Frame mainFrame;
   private Label headerLabel;
   private Panel controlPanel;
   public MyCanvas canv;
   public AwtControlDemo(){
      prepareGUI();
   }

   private void prepareGUI(){
      mainFrame = new Frame("Java AWT Examples");
      mainFrame.setSize(1000,1000);
      mainFrame.setLayout(new GridLayout(1, 1));
      
      mainFrame.addWindowListener(new WindowAdapter() {
         public void windowClosing(WindowEvent windowEvent){
            System.exit(0);
         }        
      });    
  

     
   }

   public void showCanvasDemo(double[][] lat,double x,double y,double theta,PathPlanner mp, TrackerWheels t){
	   canv = new MyCanvas(lat,x,y,theta,mp,t);
	   mainFrame.addKeyListener(canv);
	   canv.requestFocusInWindow();
     mainFrame.add(canv);
      mainFrame.setVisible(true);  
   } 

class MyCanvas extends JPanel implements KeyListener{
	   public int framenum = 0;
	   public double[][] pts;
	  public double x,y,theta;
	  public PathPlanner values;
	  public TrackerWheels trackerclass;
	  public  LinkedList<Segment> robotTrace;
	  public LinkedList<Point> lastSeen;
	  public double lastY;
	  public double lastX;
      public MyCanvas (double[][] latticeSet,double xa, double ya, double thet,PathPlanner t,TrackerWheels trac) {
    	 values = t;
    	 setBackground(Color.WHITE);
         setSize(1000, 1000);
         pts = latticeSet;
         x = xa;
         y = ya;
         theta = thet;
         trackerclass = trac;
         lastX = xa;
         lastY = ya;
         lastSeen = new LinkedList<Point>();
         lastSeen.add(new Point(xa,ya));
         
      }
      
      public void keyPressed(KeyEvent e) {
    	  //System.out.println("keyprseed.");
    	  //System.out.println(e.getKeyChar()+" printed.");
    	  switch(e.getKeyChar()) {
    	  	case 'w':
    	  		x += 3*Math.cos(-Math.toRadians(theta));
    	  		y -= 3*Math.sin(-Math.toRadians(theta));
    	  		
    	  		break;
    	  	case 'a':
    	  		theta  -= 2;
    	  		break;
    	  	case 'd':
    	  		theta += 2;
    	  		break;
    	  	case 's':
    	  		x -= 3*Math.cos(-Math.toRadians(theta));
    	  		y += 3*Math.sin(-Math.toRadians(theta));
    	  		break;
    	  }
    	 // System.out.printf("/* New Orientation %s / %s / %s */ ",x,y,theta);
    	  // System.out.printf("Vector: "+Math.cos(-Math.toRadians(theta)) + " "+Math.sin(-Math.toRadians(theta)) + " \n");
    	  trackerclass.pos = new double[] {x,y,theta};
    	  values.runPath();
    	  
    	  
      }
      
      @Override
      public void paintComponent (Graphics g) {
    // System.out.printf("[Frame #%s]repainting: %s %s %s\n",framenum,x,y,theta);
         framenum++;
    	  Graphics2D g2;
         

         super.paintComponent(g);
    	 g2 = (Graphics2D) g;
    	 g2.setColor(Color.WHITE);
         g2.fillRect(0,0,this.getWidth(),this.getHeight()); //<-- clear the background
         g2.setColor(Color.BLACK);
        
         //g2.drawLine((int)x, (int)y, (int)pts[0][0], (int)pts[0][1]);
         for(int i = 1; i < pts.length;i++) {
        	 g2.drawLine((int)pts[i-1][0],(int)pts[i-1][1],(int)pts[i][0],(int)pts[i][1]);
         }
         double rad = Math.toRadians(trackerclass.pos[2]);
         int dx = (int) (values.targetX - x);
         int dy = (int) (values.targetY - y);
         g2.drawLine((int) values.midpoint[0] ,(int) values.midpoint[1], (int)values.midpoint[0] - 100 * dy, (int)values.midpoint[1] + 100*dx);
       //  g2.drawLine( (int) values.midpoint[0] ,(int) values.midpoint[1], (int)(values.midpoint[0] + 1000 * Math.cos(rad)),  (int)(values.midpoint[1] + (1000 * Math.sin(rad))));
         
         Point lp = lastSeen.peekLast();
         Point cur = new Point(x,y);
         if (lp.dist(cur) > 2) {
        	 lastSeen.add(cur);
         }
         while(lastSeen.size() > 10000) {
        	 lastSeen.removeFirst();
         }
         g2.setColor(Color.RED);
         for (Point last : lastSeen) {
        	 g2.drawRect((int)(last.x-1),(int) (last.y-1), 2, 2);
         }
         g2.setColor(Color.BLACK);
         for (double[] pt : pts) {
        	 Shape circle = new Ellipse2D.Double(pt[0]-3,pt[1]-3,6,6);
        	 g2.fill(circle);
         }
         
         
         Shape circle = new Ellipse2D.Double(x-10,y-10,20,20);
         double offsetx = 5*Math.cos(-Math.toRadians(theta));
         double offsety = 5*Math.sin(-Math.toRadians(theta));
    	 Shape circle2 = new Ellipse2D.Double((x+offsetx)-5, (y-offsety)-5, 10, 10);
        g2.fill(circle);
        //System.out.println(0.5*(values.targetX + x) + " "+0.5*(values.targetY + y));
        g2.drawLine((int)x, (int)y, (int)values.targetX, (int)values.targetY);
        g2.drawLine((int)(x + 1000*Math.cos(Math.toRadians(theta + 90))), (int)(y + 1000*Math.sin(Math.toRadians(theta + 90))), (int)(x - 1000*Math.cos(Math.toRadians(theta + 90))), (int)(y - 1000*Math.sin(Math.toRadians(theta + 90))));
        g2.setColor(Color.RED);
        Shape origPoint = new Ellipse2D.Double(values.originCirc[0]-5, values.originCirc[1]-5, 10,10);
        Shape op2 = new Ellipse2D.Double(values.midpoint[0]-5, values.midpoint[1]-5, 10,10);

        Shape arc = new Ellipse2D.Double(values.originCirc[0]-values.r_cent,values.originCirc[1]-values.r_cent, 2*values.r_cent,2*values.r_cent);
        g2.draw(arc);
        g2.fill(origPoint);
        g2.fill(circle2);
        g2.setColor(Color.blue);
        g2.fill(op2);
        g2.setColor(Color.GREEN);
        circle = new Ellipse2D.Double(values.targetX-5,values.targetY-5,10,10);
        g2.fill(circle);
        circle = new Ellipse2D.Double(x-values.lookAheadDistance,y-values.lookAheadDistance,values.lookAheadDistance*2,values.lookAheadDistance*2);
        g2.draw(circle);
        this.repaint();
      }

	@Override
	public void keyReleased(KeyEvent arg0) {

	}

	@Override
	public void keyTyped(KeyEvent e) {

	}

	
      
   }
}
class Point{
	public double x;
	public double y;
	public Point(double i, double j) {
		x = i;
		y = j;
	}
	public double dist(Point nex) {
		return Math.hypot(x-nex.x, y-nex.y);
	}
}