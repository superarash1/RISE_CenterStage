package org.firstinspires.ftc.robotcontroller.Math;

import org.firstinspires.ftc.robotcontroller.Math.Vectors.Vector2D;

public class MathUtils {
    public static boolean epsilonEquals(double val1, double val2){
        return Math.abs(val1 - val2) < 1e-6;
    }

    public static Vector2D toPolar(double x, double y){
        double r = Math.sqrt((x * x) + (y * y));
        double theta = Math.atan2(y, x);
        return new Vector2D(r, theta);
    }
//
    public static double getRadRotDist(double start, double end){
        double diff = (end - start + Math.PI) % (2 * Math.PI) - Math.PI;
        return diff < -Math.PI ? (diff + (Math.PI * 2)) : diff;
    }
//
//    public static Angle getRotDist(Angle start, Angle end){
//        return Angle.radians(MathUtils.getRadRotDist(start.radians(), end.radians()));
//    }
//
    public static double sign(double in){
        if(epsilonEquals(in, 0.0)) {
            return 0;
        }
        return in/Math.abs(in);
    }
//
    public static double signedMax(double val, double compare){
        if(Math.abs(val) < Math.abs(compare)){
            return sign(val) * compare;
        }
        return val;
    }
//
    public static boolean inBetween(Vector2D start, Vector2D end, Vector2D point) {
        double minX = Math.min(start.A, end.A);
        double minY = Math.min(start.B, end.B);
        double maxX = Math.max(start.A, end.A);
        double maxY = Math.max(start.B, end.B);
        return (point.A < maxX) && (point.A > minX) && (point.B < maxY) && (point.B > minY);
    }
////
//    public static ArrayList<Vector> approxCurve(Vector start, Vector end, Vector control, double numSegments){
//        ArrayList<Vector> coordinates = new ArrayList<Vector>();
//
//        coordinates.add(start);
//
//        double s  = 0;
//        double t = 1;
//
//        while (s < t) {
//            s += 1/numSegments;
//            double controlParameter = (1 - s);
//            Vector Q_0 = new Vector(new double[]{controlParameter * start.get(0), controlParameter * start.get(1)}).add(new Vector(new double[]{s * control.get(0), s * control.get(1)}));
//            Vector Q_1 = new Vector(controlParameter * control.getA(), controlParameter * control.getB()).add(new Vector(s * end.getA(), s * end.getB()));
//            Vector R_0 = new Vector(controlParameter * Q_0.getA(), controlParameter * Q_0.getB()).add(new Vector(s * Q_1.getA(), s * Q_1.getB()));
//            coordinates.add(R_0);
//        }
//        coordinates.remove(coordinates.size()-1);
//        coordinates.add(end);
//        return coordinates;
//    }
//
//    public static ArrayList<Vector2> approxCurve(Vector2 start, Vector2 end, Vector2 control) {
//        return approxCurve(start, end, control, (start.distanceTo(control) + control.distanceTo(end)) * 0.1);
//    }
//
//    public static double arcLength(ArrayList<Vector2> lines) {
//        double dist = 0;
//        for(int i = 1; i < lines.size(); i ++) {
//            dist += lines.get(i-1).distanceTo(lines.get(i));
//        }
//        return dist;
//    }
//
//    public static Vector2 getClosestPoint(Vector2 line1point1, Vector2 line1point2, Vector2 line2point){
//        double dx = line1point1.getA() - line1point2.getA();
//        double dy = line1point1.getB() - line1point2.getB();
//
//        double line1slope = 0;
//
//        double x = 0;
//        double y = 0;
//
//        if(dx == 0 && dy == 0){
//            return line1point1;
//        }else if(dx == 0){
//            x = line1point1.getA();
//            y = line2point.getB();
//            //return new Vector2(line1point1.getA(), line2point.getB());
//        }else if(dy == 0){
//            x = line2point.getA();
//            y = line1point1.getB();
//            //return new Vector2(line2point.getA(), line1point1.getB());
//        }else{
//            line1slope = dy/dx;
//
//            double line2slope = -1/line1slope;
//
//            x = ((-line2slope * line2point.getA()) + line2point.getB() + (line1slope * line1point1.getA()) - line1point1.getB())/(line1slope - line2slope);
//
//            y = line1slope * (x - line1point1.getA()) + line1point1.getB();
//        }
//
//        double minX = Math.min(line1point1.getA(), line1point2.getA());
//        double maxX = Math.max(line1point1.getA(), line1point2.getA());
//
//        double minY = Math.min(line1point1.getB(), line1point2.getB());
//        double maxY = Math.max(line1point1.getB(), line1point2.getB());
//
//        if(x < minX || x > maxX || y < minY || y > maxY) {
//            if(line1point1.distanceTo(line2point) < line1point2.distanceTo(line2point)) {
//                return line1point1;
//            }else {
//                return line1point2;
//            }
//        }
//
//        return new Vector2(x, y);
//    }
//
//    public static Vector2 toPolar(Vector2 pos){
//        return toPolar(pos.getA(), pos.getB());
//    }
//
//    public static double clamp(double val, double min, double max){
//        return Math.max(min, Math.min(max, val));
//    }
//
//    public static Vector2 toCartesian(double r, double theta){
//        return new Vector2(r * Math.cos(theta), r * Math.sin(theta));
//    }
//
//    public static Vector2 toCartesian(Vector2 pos){
//        return toCartesian(pos.getA(), pos.getB());
//    }


    /**
     * Puts a matrix into reduced row echelon form
     *
     * @param matrix input matrix
     *
     * @return 2D result matrix
     */
    public static double[][] rref(double[][] matrix){
        int columnIndex = 0;
        int cursor;

        // number of rows and columns in matrix
        int getRowSize = matrix.length;
        int getColumnSize = matrix[0].length;


        loop:
        for(int rowIndex = 0; rowIndex < getRowSize; rowIndex++){
            if(getColumnSize <= columnIndex){
                break;
            }
            cursor = rowIndex;
            while(matrix[cursor][columnIndex] == 0){
                cursor++;
                if(getRowSize == cursor){
                    cursor = rowIndex;
                    columnIndex++;
                    if(getColumnSize == columnIndex){
                        break loop;
                    }
                }

            }

            matrix = rowSwap(matrix, cursor, rowIndex);
            if(matrix[rowIndex][columnIndex] != 0){
                matrix = rowScale(matrix, rowIndex, (1/matrix[rowIndex][columnIndex]));
            }

            for(cursor = 0; cursor < getRowSize; cursor++){
                if(cursor != rowIndex){
                    matrix = rowAddScale(matrix, rowIndex, cursor,((-1) * matrix[cursor][columnIndex]));
                }
            }columnIndex++;
        }return matrix;
    }

    /**
     * Swap positions of 2 rows
     *
     * @param matrix matrix before row additon
     * @param rowIndex1 int index of row to swap
     * @param rowIndex2 int index of row to swap
     *
     * @return matrix after row swap
     */
    private static double[][] rowSwap(double[][] matrix, int rowIndex1,
                                      int rowIndex2){
        // number of columns in matrix
        int numColumns = matrix[0].length;

        // holds number to be swapped
        double hold;

        for(int k = 0; k < numColumns; k++){
            hold = matrix[rowIndex2][k];
            matrix[rowIndex2][k] = matrix[rowIndex1][k];
            matrix[rowIndex1][k] = hold;
        }

        return matrix;
    }

    /**
     * Adds 2 rows together row2 = row2 + row1
     *
     * @param matrix matrix before row additon
     * @param rowIndex1 int index of row to be added
     * @param rowIndex2 int index or row that row1 is added to
     *
     * @return matrix after row addition
     */
    private static double[][] rowAdd(double[][] matrix, int rowIndex1,
                                     int rowIndex2){
        // number of columns in matrix
        int numColumns = matrix[0].length;

        for(int k = 0; k < numColumns; k++){
            matrix[rowIndex2][k] += matrix[rowIndex1][k];
        }

        return matrix;
    }

    /**
     * Multiplies a row by a scalar
     *
     * @param matrix matrix before row additon
     * @param rowIndex int index of row to be scaled
     * @param scalar double to scale row by
     *
     * @return matrix after row scaling
     */
    private static double[][] rowScale(double[][] matrix, int rowIndex,
                                       double scalar){
        // number of columns in matrix
        int numColumns = matrix[0].length;

        for(int k = 0; k < numColumns; k++){
            matrix[rowIndex][k] *= scalar;
        }

        return matrix;
    }

    /**
     * Adds a row by the scalar of another row
     * row2 = row2 + (row1 * scalar)
     * @param matrix matrix before row additon
     * @param rowIndex1 int index of row to be added
     * @param rowIndex2 int index or row that row1 is added to
     * @param scalar double to scale row by
     *
     * @return matrix after row addition
     */
    private static double[][] rowAddScale(double[][] matrix, int rowIndex1,
                                          int rowIndex2, double scalar){
        // number of columns in matrix
        int numColumns = matrix[0].length;

        for(int k = 0; k < numColumns; k++){
            matrix[rowIndex2][k] += (matrix[rowIndex1][k] * scalar);
        }

        return matrix;
    }

    public static double millisToSec(long time){
        return (time/1000.0);
    }

    public static long secToMillis(long time){
        return time * 1000;
    }

    public static long secToNano(long time){
        return time * ((long)1_000_000_000);
    }

    public static long millisToNano(long time){
        return secToNano((long)millisToSec(time));
    }

    public static long nanoToMillis(long time){
        return secToMillis(nanoToSec(time));
    }

    public static long nanoToSec(long time){
        return time/((long)1_000_000_000);
    }

    public static double nanoToDSec(long time){
        return time/1_000_000_000.0;
    }
}
