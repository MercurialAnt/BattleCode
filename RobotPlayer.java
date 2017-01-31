
package examplefuncsplayer;
import java.util.Random;

import battlecode.common.*;

public strictfp class RobotPlayer {
    static RobotController rc;
    static Random ran = new Random();

    /**
     * run() is the method that is called when a robot is instantiated in the Battlecode world.
     * If this method returns, the robot dies!
    **/
	
		
	@SuppressWarnings("unused")
    public static void run(RobotController rc) throws GameActionException {

        // This is the RobotController object. You use it to perform actions from this robot,
        // and to get information on its current status.
        RobotPlayer.rc = rc;
        // Here, we've separated the controls into a different method for each RobotType.
        // You can add the missing ones or rewrite this into your own control structure.
        switch (rc.getType()) {
            case ARCHON:
                runArchon();
                break;
            case GARDENER:
                runGardener();
                break;
            case SCOUT:
            	runScout();
            	break;
            case SOLDIER:
                runSoldier();
                break;
            case LUMBERJACK:
                runLumberjack();
                break;
        }
	}

    static void runArchon() throws GameActionException {
        System.out.println("I'm an archon!");
        
        // The code you want your robot to perform every round should be in this loop
        while (true) {

            // Try/catch blocks stop unhandled exceptions, which cause your robot to explode
            try {
            	
            	if(rc.getTeamBullets()>3000)
            		rc.donate(rc.getTeamBullets()/2);
                // Generate a random direction
                Direction dir = randomDirection();

                // Randomly attempt to build a gardener in this direction
                if (rc.canHireGardener(dir) && Math.random() < .01) {
                    rc.hireGardener(dir);
                }

                // Move randomly
                tryMove(randomDirection());

                // Broadcast archon's location for other robots on the team to know
                MapLocation myLocation = rc.getLocation();
                rc.broadcast(0,(int)myLocation.x);
                rc.broadcast(1,(int)myLocation.y);

                // Clock.yield() makes the robot wait until the next turn, then it will perform this loop again
                Clock.yield();

            } catch (Exception e) {
                System.out.println("Archon Exception");
                e.printStackTrace();
            }
        }
    }

	static void runGardener() throws GameActionException {
        System.out.println("I'm a gardener!");
        
        Direction buildDir = null;
        float buildFloat = 0;
        // Generate a random direction
        Direction dir = randomDirection();
        
        for(int i=0;i<5;i++)
        {
        	Direction test = new Direction((float)(i*Math.PI/3));
        	if(rc.canPlantTree(test))
        	{
        		buildFloat = test.getAngleDegrees();
        		buildDir = new Direction(buildFloat);
        		break;
        	}
        }
        // The code you want your robot to perform every round should be in this loop
        while (true) {

            // Try/catch blocks stop unhandled exceptions, which cause your robot to explode
            try {

                // Listen for home archon's location
            	if(rc.getTeamBullets()>3000)
            		rc.donate(rc.getTeamBullets()/2);
            	int xPos = rc.readBroadcast(0);
                int yPos = rc.readBroadcast(1);
                MapLocation archonLoc = new MapLocation(xPos,yPos);

                
                
                TreeInfo[] trees = rc.senseNearbyTrees(GameConstants.INTERACTION_DIST_FROM_EDGE+1, rc.getTeam());
                
                // plant trees
                for(int i=1;i<6;i++)
                {
                	Direction treeDir = new Direction((float) ((float) buildFloat+(i*Math.PI/3)));
                	if(rc.canPlantTree(treeDir))
                	{
                		rc.plantTree(treeDir);
                	}
                }
                                
                //water trees
                int weakestIndex=0;
                float weakest=Float.MAX_VALUE;
                for(int i=0;i<trees.length;i++)
                {
                	float hp = trees[i].health;
                	if(hp <weakest)
                	{
                		weakest= hp; 
                		weakestIndex = i;
               		}
                	if(rc.canShake(trees[i].ID))
                	{
                		rc.shake(trees[i].ID);
                	}
                }
                if(rc.canWater(trees[weakestIndex].location))
                {
                	rc.water(trees[weakestIndex].ID);
                }
                
                // Randomly attempt to build a soldier or lumberjack in this direction
                if (rc.canBuildRobot(RobotType.LUMBERJACK, buildDir)&& rc.isBuildReady() ) {
                    rc.buildRobot(RobotType.LUMBERJACK, buildDir);
                    System.out.println("lumberjack");
                }
                if(rc.canBuildRobot(RobotType.SCOUT, buildDir))
                	System.out.println("trying to build a robot");
                if (rc.canBuildRobot(RobotType.SCOUT, buildDir)&& rc.isBuildReady() ) {
                    rc.buildRobot(RobotType.SCOUT, buildDir);
                    System.out.println("plane");
                }
                

                // Clock.yield() makes the robot wait until the next turn, then it will perform this loop again
                Clock.yield();

            } catch (Exception e) {
                System.out.println("Gardener Exception");
                e.printStackTrace();
            }
        }
    }
	static void runScout() throws GameActionException {
        System.out.println("I'm an Scout!");
        Team enemy = rc.getTeam().opponent();
        MapLocation[] archonLoc = rc.getInitialArchonLocations(enemy);
        int amountOfArchons = archonLoc.length, archonTarget=0;
        boolean enemyFound=false,close, neverReach=false;
        rc.broadcastBoolean(5, false);
        // The code you want your robot to perform every round should be in this loop
        while (true) {
        	

            // Try/catch blocks stop unhandled exceptions, which cause your robot to explode
            try {
            	
                MapLocation myLocation = rc.getLocation();

                // See if there are any nearby enemy robots
                RobotInfo[] robots = rc.senseNearbyRobots(-1, enemy);
                for(int i=0;i<robots.length;i++)
                {
                	if(robots[i].type.equals(RobotType.GARDENER))
                	{
                		MapLocation gard = robots[i].getLocation();
                		rc.broadcastFloat(3,gard.x );
                		rc.broadcastFloat(4, gard.y);
                		rc.broadcastBoolean(5, true);
                		break;
                	}
                }
                MapLocation first = new MapLocation(rc.readBroadcastFloat(3),rc.readBroadcastFloat(4));
                
                
                	
                
                Direction enemyDir = null;
                
                // If there are some...
                if (robots.length > 0) {
                	enemyFound=true;
                	enemyDir =rc.getLocation().directionTo(robots[0].location);
                	
                    // And we have enough bullets, and haven't attacked yet this turn...
                    if (rc.canFireSingleShot()) {
                        // ...Then fire a bullet in the direction of the enemy.
                        rc.fireSingleShot(enemyDir);
                    }
                }
                else enemyFound=false;
                
                MapLocation archonLocTar = archonLoc[archonTarget];
                close = close(rc.getType(),rc.getLocation(), archonLocTar);
                if(close)
                {
                	neverReach = true;
                }
                
                // Move randomly
                if(enemyFound)
                {
                	tryMove(enemyDir.opposite());
                	tryMove(enemyDir);
                	archonTarget= ran.nextInt(amountOfArchons);
                }
                else if(close && !archonNear(robots))
                {
                	archonTarget = ran.nextInt(amountOfArchons);
                	tryMove(rc.getLocation().directionTo(archonLoc[archonTarget]));
                }
                else if(!rc.readBroadcastBoolean(5))
                {
                	tryMove(rc.getLocation().directionTo(archonLoc[archonTarget]));
                }
                else
                tryMove(rc.getLocation().directionTo(first));

                // Clock.yield() makes the robot wait until the next turn, then it will perform this loop again
                Clock.yield();

            } catch (Exception e) {
                System.out.println("Scout Exception");
                e.printStackTrace();
            }
        }
    }
	static boolean close(RobotType rob, MapLocation map, MapLocation map2)
	{
		float distance = map.distanceTo(map2);
		return distance< rob.sensorRadius ? true: false;
	}
	static boolean archonNear(RobotInfo[] robList)
	{
		for(int i=0; i<robList.length;i++)
		{
			if(robList[i].type == RobotType.ARCHON)
				return true;
		}
		return false;
	}

    static void runSoldier() throws GameActionException {
        System.out.println("I'm an soldier!");
        Team enemy = rc.getTeam().opponent();

        // The code you want your robot to perform every round should be in this loop
        while (true) {

            // Try/catch blocks stop unhandled exceptions, which cause your robot to explode
            try {
                MapLocation myLocation = rc.getLocation();

                // See if there are any nearby enemy robots
                RobotInfo[] robots = rc.senseNearbyRobots(-1, enemy);

                // If there are some...
                if (robots.length > 0) {
                    // And we have enough bullets, and haven't attacked yet this turn...
                    if (rc.canFireSingleShot()) {
                        // ...Then fire a bullet in the direction of the enemy.
                        rc.fireSingleShot(rc.getLocation().directionTo(robots[0].location));
                    }
                }
                // Move randomly
                tryMove(randomDirection());

                // Clock.yield() makes the robot wait until the next turn, then it will perform this loop again
                Clock.yield();

            } catch (Exception e) {
                System.out.println("Soldier Exception");
                e.printStackTrace();
            }
        }
    }

    static void runLumberjack() throws GameActionException {
        System.out.println("I'm a lumberjack!");
        Team enemy = rc.getTeam().opponent();
        boolean healthLow=false;
        rc.broadcastBoolean(5, false);
        // The code you want your robot to perform every round should be in this loop
        while (true) {
        	MapLocation[] archonLoc= rc.getInitialArchonLocations(enemy);

            // Try/catch blocks stop unhandled exceptions, which cause your robot to explode
            try {
            	if(rc.getHealth()<RobotType.LUMBERJACK.maxHealth/2)
            	{
            		healthLow=true;
            	}

                // See if there are any enemy robots within striking range (distance 1 from lumberjack's radius)
                RobotInfo[] robots = rc.senseNearbyRobots(GameConstants.LUMBERJACK_STRIKE_RADIUS, enemy);
                TreeInfo[] trees = rc.senseNearbyTrees(GameConstants.LUMBERJACK_STRIKE_RADIUS, Team.NEUTRAL);
                TreeInfo[] enemyTrees = rc.senseNearbyTrees(GameConstants.LUMBERJACK_STRIKE_RADIUS, enemy);
                
                if((robots.length > 0 || trees.length > 0 || enemyTrees.length>0) && !rc.hasAttacked()) {
                    // Use strike() to hit all nearby robots!
                    rc.strike();
                } 
                    // No close robots, so search for trees within lumberjack sensor radius
                    trees = rc.senseNearbyTrees(RobotType.LUMBERJACK.sensorRadius, Team.NEUTRAL);
                    robots = rc.senseNearbyRobots(RobotType.LUMBERJACK.sensorRadius, enemy);
                    enemyTrees = rc.senseNearbyTrees(RobotType.LUMBERJACK.sensorRadius, enemy);
                    RobotInfo[] friendRobots = rc.senseNearbyRobots(GameConstants.LUMBERJACK_STRIKE_RADIUS, enemy);
                    
                    for(int i=0;i<robots.length;i++)
                    {
                    	if(robots[i].type.equals(RobotType.GARDENER))
                    	{
                    		MapLocation gard = robots[i].getLocation();
                    		rc.broadcastFloat(3,gard.x );
                    		rc.broadcastFloat(4, gard.y);
                    		rc.broadcastBoolean(5, true);
                    		break;
                    	}
                    }
                    MapLocation first = new MapLocation(rc.readBroadcastFloat(3),rc.readBroadcastFloat(4));
                    
                    if(friendRobots.length >0)
                    {
                    	MapLocation myLocation = rc.getLocation();
                        MapLocation friendLocation = friendRobots[0].location;
                        Direction awayFriend = myLocation.directionTo(friendLocation).opposite();

                        tryMove(awayFriend);
                    }
                    else if(trees.length > 0 && !healthLow) {
                        MapLocation myLocation = rc.getLocation();
                        MapLocation treeLocation = trees[0].location;
                        Direction toTree = myLocation.directionTo(treeLocation);

                        tryMove(toTree);
                    }
                    else if(robots.length>0){
                    	MapLocation myLocation = rc.getLocation();
                        MapLocation robotLocation = robots[0].location;
                        Direction toRobot = myLocation.directionTo(robotLocation);
                        
                        tryMove(toRobot);
                    }
                    else if(enemyTrees.length>0 && !healthLow){
                        MapLocation myLocation = rc.getLocation();
                        MapLocation treeLocation = enemyTrees[0].location;
                        Direction toTree = myLocation.directionTo(treeLocation);

                        tryMove(toTree);
                    }	
                    else if(rc.readBroadcastBoolean(5))
                    {
                    	tryMove(rc.getLocation().directionTo(first));
                    }
                    else{
                        // Move Randomly
                        tryMove( rc.getLocation().directionTo(archonLoc[ran.nextInt(archonLoc.length)]));
                    }
                

                // Clock.yield() makes the robot wait until the next turn, then it will perform this loop again
                Clock.yield();

            } catch (Exception e) {
                System.out.println("Lumberjack Exception");
                e.printStackTrace();
            }
        }
    }

    /**
     * Returns a random Direction
     * @return a random Direction
     */
    static Direction randomDirection() {
        return new Direction((float)Math.random() * 2 * (float)Math.PI);
    }

    /**
     * Attempts to move in a given direction, while avoiding small obstacles directly in the path.
     *
     * @param dir The intended direction of movement
     * @return true if a move was performed
     * @throws GameActionException
     */
    static boolean tryMove(Direction dir) throws GameActionException {
        return tryMove(dir,20,5);
    }

    /**
     * Attempts to move in a given direction, while avoiding small obstacles direction in the path.
     *
     * @param dir The intended direction of movement
     * @param degreeOffset Spacing between checked directions (degrees)
     * @param checksPerSide Number of extra directions checked on each side, if intended direction was unavailable
     * @return true if a move was performed
     * @throws GameActionException
     */
    static boolean tryMove(Direction dir, float degreeOffset, int checksPerSide) throws GameActionException {

        // First, try intended direction
        if (rc.canMove(dir)) {
            rc.move(dir);
            return true;
        }

        // Now try a bunch of similar angles
        boolean moved = false;
        int currentCheck = 1;

        while(currentCheck<=checksPerSide) {
            // Try the offset of the left side
            if(rc.canMove(dir.rotateLeftDegrees(degreeOffset*currentCheck))) {
                rc.move(dir.rotateLeftDegrees(degreeOffset*currentCheck));
                return true;
            }
            // Try the offset on the right side
            if(rc.canMove(dir.rotateRightDegrees(degreeOffset*currentCheck))) {
                rc.move(dir.rotateRightDegrees(degreeOffset*currentCheck));
                return true;
            }
            // No move performed, try slightly further
            currentCheck++;
        }

        // A move never happened, so return false.
        return false;
    }

    /**
     * A slightly more complicated example function, this returns true if the given bullet is on a collision
     * course with the current robot. Doesn't take into account objects between the bullet and this robot.
     *
     * @param bullet The bullet in question
     * @return True if the line of the bullet's path intersects with this robot's current position.
     */
    static boolean willCollideWithMe(BulletInfo bullet) {
        MapLocation myLocation = rc.getLocation();

        // Get relevant bullet information
        Direction propagationDirection = bullet.dir;
        MapLocation bulletLocation = bullet.location;

        // Calculate bullet relations to this robot
        Direction directionToRobot = bulletLocation.directionTo(myLocation);
        float distToRobot = bulletLocation.distanceTo(myLocation);
        float theta = propagationDirection.radiansBetween(directionToRobot);

        // If theta > 90 degrees, then the bullet is traveling away from us and we can break early
        if (Math.abs(theta) > Math.PI/2) {
            return false;
        }

        // distToRobot is our hypotenuse, theta is our angle, and we want to know this length of the opposite leg.
        // This is the distance of a line that goes from myLocation and intersects perpendicularly with propagationDirection.
        // This corresponds to the smallest radius circle centered at our location that would intersect with the
        // line that is the path of the bullet.
        float perpendicularDist = (float)Math.abs(distToRobot * Math.sin(theta)); // soh cah toa :)

        return (perpendicularDist <= rc.getType().bodyRadius);
    }
}
