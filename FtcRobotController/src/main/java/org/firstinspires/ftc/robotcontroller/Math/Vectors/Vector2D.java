package org.firstinspires.ftc.robotcontroller.Math.Vectors;

public class Vector2D {
    private static final Vector2D ZERO = new Vector2D(0, 0);

    public double A, B;

    public Vector2D(double A, double B){
        this.A = A;
        this.B = B;
    }

    public void set_î(double A) {
        this.A = A;
    }

    public void set_Ĵ(double B) {
        this.B = B;
    }

    public void set(double A, double B) {
        this.A = A;
        this.B = B;
    }

    public void set(Vector2D v) {
        A = v.A;
        B = v.B;
    }

    public Vector2D clone(){
        return new Vector2D(A, B);
    }

    public Vector2D add(Vector2D v) {
        return new Vector2D(A +v.A, B +v.B);
    }

    public Vector2D add(double A, double B) {
        return new Vector2D(this.A +A, this.B +B);
    }

    public Vector2D subtract(Vector2D v) {
        return new Vector2D(A-v.A, B-v.B);
    }

    public Vector2D subtract(double A, double B) {
        return new Vector2D(this.A-A, this.B-B);
    }

    //TODO: add a dot product and cross product method

    public double findDistance(Vector2D v){
        return Math.sqrt(Math.pow(A - v.A, 2) + Math.pow(B - v.B, 2));
    }

    public Vector2D scale(double scalar){
        return new Vector2D(A*scalar, B*scalar);
    }

    public Vector2D normalize() {
        return scale(1/getMagnitude());
    }

    public double getMagnitude(){
        return Math.sqrt((A * A) + (B * B));
    }

    public double DotProduct(double A, double B){
        return this.A*A + this.B*B;
    }

    public static Vector2D ZERO(){
        return ZERO.clone();
    }


//
//    public void set(Vector2 v, double c) {
//        this.a = v.getA();
//        this.b = v.getB();
//        this.c = c;
//    }

//    public Vector2 getVector2(){
//        return new Vector2(a, b);
//    }
//

    //
//    public boolean equals(Vector3 vector3){
//        return (vector3.a == a) && (vector3.b == b) && (vector3.c == c);
//    }
//
//    public Vector4 getVector4(double d){
//        return new Vector4(a, b, c, d);
//    }
//
    @Override
    public String toString() {
        return A + ", " + B;
    }

}
