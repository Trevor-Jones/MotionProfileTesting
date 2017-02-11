package core;

import core.PathPlanner.LeftRightProfile;
import core.WaypointSequence;

/**
 *
 * @author Jared341
 */
public class Main {
	
	public static void main(String[] args) {
		PathPlanner planner = new PathPlanner(.01, 8, 50, 10, 2);

		WaypointSequence p = new WaypointSequence(10);
		p.addWaypoint(new WaypointSequence.Waypoint(0, 0, 0));
		p.addWaypoint(new WaypointSequence.Waypoint(14, 7, Math.PI / 6));
		
		LeftRightProfile profile = planner.generatePath(p);
		System.out.println("Number of points left: " + profile.left.length);
		System.out.println("Number of points right: " + profile.right.length);
		
		for(int i = 0; i < profile.left.length; i++) {
			System.out.println("{Pos: " + profile.left[i][0] + ", Vel: " + profile.left[i][1] + ", dT: " + profile.left[i][2]);
			//System.out.print(profile.left[i][1]/60 + ",");
		}
		//System.out.println("");
		for(int i = 0; i < profile.right.length; i++) {
			//System.out.println("{Pos: " + profile.right[i][0] + ", Vel: " + profile.right[i][1] + ", dT: " + profile.right[i][2]);
			//System.out.print(profile.right[i][1]/60 + ",");
		}
	}
}
