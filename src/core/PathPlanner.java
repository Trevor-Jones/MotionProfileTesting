package core;

import core.Trajectory.Segment;

public class PathPlanner {
	
	TrajectoryGenerator.Config config = new TrajectoryGenerator.Config();
	
	public PathPlanner(double dt, double maxAcc, double maxJerk, double maxVel, double drivebaseWidth) {
		config.dt = dt;
		config.max_acc = maxAcc;
		config.max_jerk = maxJerk;
		config.max_vel = maxVel;
		config.drivebase_width = drivebaseWidth;
	}
	
	public class LeftRightProfile {
		public double[][] left,right;
		
		public LeftRightProfile(double[][]left, double[][] right) {
			this.left = left;
			this.right = right;
		}
	}
	
	
	public LeftRightProfile generatePath(WaypointSequence p) {
		double time = System.currentTimeMillis();
		
		Path path = PathGenerator.makePath(p, config, config.drivebase_width, "path");
		Trajectory.Pair trajectory = path.getPair();
		
		double[][] leftPosVel = new double[trajectory.left.getNumSegments()][3];
		double[][] rightPosVel = new double[trajectory.right.getNumSegments()][3];
		
		Segment seg;
		
		for(int i = 0; i < trajectory.left.getNumSegments(); i++) {
			seg = trajectory.left.getSegment(i);
			leftPosVel[i][0] = seg.pos;
			leftPosVel[i][1] = seg.vel * 60; // Talon velocity is per minute
			leftPosVel[i][2] = seg.dt;
		}
		
		for(int i = 0; i < trajectory.right.getNumSegments(); i++) {
			seg = trajectory.right.getSegment(i);
			rightPosVel[i][0] = seg.pos;
			rightPosVel[i][1] = seg.vel * 60; // Taloon velocity is per minute
			rightPosVel[i][2] = seg.dt;
		}
		
		System.out.println("TOTAL TIME: " + (System.currentTimeMillis() - time));
		
		LeftRightProfile profile = new LeftRightProfile(leftPosVel, rightPosVel);
		return profile;
	}
}
