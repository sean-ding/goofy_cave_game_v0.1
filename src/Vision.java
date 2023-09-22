public class Vision{
    private static final double VISION_BOUNDS = 0.49;
    private static double slope(double y1, double x1, double y2, double x2){
        try {
            double m = (y2 - y1) / (x2 - x1);
            return m;
        } catch (Exception e) {
            throw new ArithmeticException("Slope is undefined");
        }
    }

    private static double pointDistance(double[] p1, double[] p2){
        double a = Math.abs(p1[0] - p2[0]);
        double b = Math.abs(p1[1] - p2[1]);
        double c = Math.sqrt(a*a + b*b);
        return c;
    }

    private static double[] lineIntersection(double m1, double b1, double m2, double b2){
        double x = (b2-b1) / (m1-m2);
        double y = (m1 * x) + b1;
        return new double[]{y, x};
    }

    private static double perpendicularSlope(double m){
        try {
            return (1 / m) * -1;
        } catch (Exception e) {
            throw new ArithmeticException("Slope is undefined");
        }
    }

    private static int[] closestPoint(int[] p1, int[][] p_arr){
        int[] closest = p_arr[0];
        double closestDistance = pointDistance(Hutil.doubleArr(p1), Hutil.doubleArr(closest));
        for(int[] i : p_arr){
            double d = pointDistance(Hutil.doubleArr(p1), Hutil.doubleArr(i));
            if(d < closestDistance);
            closestDistance = d;
            closest = i;
        }
        return closest;
    }

    private static double yInt(int y, int x, double m){
        return y - (m * x);
    }
    private static double yInt(double y, double x, double m){
        return y - (m * x);
    }

    // separate blocked method, this one just ignores self blocking, used for ray casts where you only want to know what it hits
    private static boolean blocked(double[] intersect, int[] blocker){
        double sightlineDistanceToWall = pointDistance(intersect, Hutil.doubleArr(blocker)); // a wall can be seen as a circle with a radius of 0.5, enough to touch its neighbors
        return sightlineDistanceToWall < 0.5;
    }
    private static boolean blocked(double[] intersect, int[] blocker, double y, double x){
        double sightlineDistanceToWall = pointDistance(intersect, Hutil.doubleArr(blocker)); // a wall can be seen as a circle with a radius of 0.5, enough to touch its neighbors
        boolean sightlineGoesThroughWall = (sightlineDistanceToWall < 0.5);

        boolean selfBlocking = (Hutil.inRange(y, blocker[0]-VISION_BOUNDS, blocker[0]+VISION_BOUNDS) && Hutil.inRange(x, blocker[1]-VISION_BOUNDS, blocker[1]+VISION_BOUNDS));

        return sightlineGoesThroughWall && !selfBlocking;
    }

    public static boolean canSee(int y, int x, int[] observerPosition, String[][] grid, int RENDER_DISTANCE){
        // checks if there is a sightline to any of the 4 edges
        if (getRayCastHitByGrid(y-VISION_BOUNDS, x, observerPosition, grid, RENDER_DISTANCE) == null) {
            return true;
        }
        if (getRayCastHitByGrid(y+VISION_BOUNDS, x, observerPosition, grid, RENDER_DISTANCE) == null) {
            return true;
        }
        if (getRayCastHitByGrid(y, x+VISION_BOUNDS, observerPosition, grid, RENDER_DISTANCE) == null) {
            return true;
        }
        if (getRayCastHitByGrid(y, x-VISION_BOUNDS, observerPosition, grid, RENDER_DISTANCE) == null) {
            return true;
        }
	    return false;
    }

    public static int[] getRayCastHitByGrid(double targetY, double targetX, int[] origin, String[][] grid, int castLength) {
        double[] intersect = {};
        int castGridLength = castLength * 2 + 1;
        if (castLength == -1) {
            castGridLength = grid.length;
        }

        // works based on grade 9 analytical geometry
        // m is slope and b is y-intercept

        boolean m_isUndefined = (targetX - origin[1] == 0);
        if(!m_isUndefined){

            double m = slope(targetY, targetX, origin[0], origin[1]);
            double b = yInt(targetY, targetX, m);
            // check for walls within camera
            for (int wallY = 0; wallY < castGridLength; wallY++) {
                for (int wallX = 0; wallX < castGridLength; wallX++) {
                    // convert relative position to grid if cast distance is specified. same as Main.toGrid()
                    int gridWallY = (origin[0] - castLength) + wallY;
                    int gridWallX = (origin[1] - castLength) + wallX;
                    if (castLength == -1) {
                        gridWallX = wallX;
                        gridWallY = wallY;
                    }

                   // checks if wall is between observer and target (ignored if it won't be in the way)
                    if(Hutil.inRange(gridWallY, targetY, origin[0]) && Hutil.inRange(gridWallX, targetX, origin[1])) {
                        if(grid[gridWallY][gridWallX] == "X"){
                            int[] blocker = {gridWallY, gridWallX}; // position of wall that could block sightline
                        
                            if(m != 0){

                                // line originating from wall running perpendicular to sightline
                                double m2 = perpendicularSlope(m);
                                double b2 = yInt(gridWallY, gridWallX, m2);

                                intersect = lineIntersection(m, b, m2, b2); // intersection of sightline and line from wall to sightline

                                if(blocked(intersect, blocker, targetY, targetX)){
                                    return blocker;
                                }

                            }
                            else{
                                intersect = new double[]{targetY, gridWallX};

                                if(blocked(intersect, blocker, targetY, targetX)){
                                    return blocker;
                                }
                            }
                        }
                    }
                }
            }

        }
        else{
            // check for walls within camera (for undefined slope, i.e. vertical line)
            for (int wallY = 0; wallY < castGridLength; wallY++) {
                for (int wallX = 0; wallX < castGridLength; wallX++) {
                    // convert relative position to grid if cast distance is specified. same as Main.toGrid()
                    int gridWallY = (origin[0] - castLength) + wallY;
                    int gridWallX = (origin[1] - castLength) + wallX;
                    if (castLength == -1) {
                        gridWallX = wallX;
                        gridWallY = wallY;
                    }

                    if(Hutil.inRange(gridWallY, targetY, origin[0]) && Hutil.inRange(gridWallX, targetX, origin[1])) {

                        if(grid[gridWallY][gridWallX] == "X"){
                            int[] blocker = {gridWallY, gridWallX}; // position of wall that could block sightline
                            intersect = new double[]{gridWallY, targetX}; // intersection of sightline and line from wall to sightline

                            if(blocked(intersect, blocker, targetY, targetX)){
                                return blocker;
                            }
                        }
                    }
                }
            }
        }
        return null;
    }
    public static int[] getRayCastHitBySlope(double m, boolean positive, boolean undefined, int[] origin, String[][] grid, int castLength) {
        double[] intersect = {};
        int castGridLength = castLength * 2 + 1;
        if (castLength == -1) {
            castGridLength = grid.length;
        }

        // works based on grade 9 analytical geometry
        // m is slope and b is y-intercept

        if(!undefined){
            double b = yInt(origin[0], origin[1], m);
            // check for walls within camera
            for (int wallY = 0; wallY < castGridLength; wallY++) {
                for (int wallX = 0; wallX < castGridLength; wallX++) {
                    // convert relative position to grid if cast distance is specified. same as Main.toGrid()
                    int gridWallY = (origin[0] - castLength) + wallY;
                    int gridWallX = (origin[1] - castLength) + wallX;
                    if (castLength == -1) {
                        gridWallX = wallX;
                        gridWallY = wallY;
                    }

                    // checks if wall is between origin and target (ignored if it won't be in the way)
                    boolean inWay = false;
                    if (m == 0) {
                        inWay = (gridWallY == origin[0]);
                    }
                    else if (positive && m > 0) {
                        inWay = (gridWallY > origin[0] && gridWallX > origin[1]);
                    }
                    else if (positive && m < 0) {
                        inWay = (gridWallY > origin[0] && gridWallX < origin[1]);
                    }
                    else if (!positive && m > 0) {
                        inWay = (gridWallY < origin[0] && gridWallX < origin[1]);
                    }
                    else if (!positive && m < 0) {
                        inWay = (gridWallY < origin[0] && gridWallX > origin[1]);
                    }

                    if(inWay) {
                        if(grid[gridWallY][gridWallX] == "X"){
                            int[] blocker = {gridWallY, gridWallX}; // position of wall that could block sightline

                            if(m != 0){

                                // line originating from wall running perpendicular to sightline
                                double m2 = perpendicularSlope(m);
                                double b2 = yInt(gridWallY, gridWallX, m2);

                                intersect = lineIntersection(m, b, m2, b2); // intersection of sightline and line from wall to sightline

                                if(blocked(intersect, blocker)){
                                    return blocker;
                                }

                            }
                            else{
                                intersect = new double[]{targetY, gridWallX};

                                if(blocked(intersect, blocker, targetY, targetX)){
                                    return blocker;
                                }
                            }
                        }
                    }
                }
            }

        }
        else{
            // check for walls within camera (for undefined slope, i.e. vertical line)
            for (int wallY = 0; wallY < castGridLength; wallY++) {
                for (int wallX = 0; wallX < castGridLength; wallX++) {
                    // convert relative position to grid if cast distance is specified. same as Main.toGrid()
                    int gridWallY = (origin[0] - castLength) + wallY;
                    int gridWallX = (origin[1] - castLength) + wallX;
                    if (castLength == -1) {
                        gridWallX = wallX;
                        gridWallY = wallY;
                    }

                    if(Hutil.inRange(gridWallY, targetY, origin[0]) && Hutil.inRange(gridWallX, targetX, origin[1])) {

                        if(grid[gridWallY][gridWallX] == "X"){
                            int[] blocker = {gridWallY, gridWallX}; // position of wall that could block sightline
                            intersect = new double[]{gridWallY, targetX}; // intersection of sightline and line from wall to sightline

                            if(blocked(intersect, blocker, targetY, targetX)){
                                return blocker;
                            }
                        }
                    }
                }
            }
        }
        return null;
    }
    public static int[][] rayCast(int[] start, int[] end, String[][] grid, int castLength) {
        int diagonalDistance = Math.max(Math.abs(start[0] - end[0]), Math.abs(start[1] - end[1]));
        if (diagonalDistance == 0 || diagonalDistance == 1) {
            return null;
        }
        int[] rayCastEnd = getRayCastHit(end[1], end[0], start, grid, castLength);
        if (rayCastEnd == null) {

        }
        int[][] line;

    }
}