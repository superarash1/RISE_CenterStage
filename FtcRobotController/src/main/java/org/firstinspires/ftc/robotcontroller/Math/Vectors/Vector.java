package org.firstinspires.ftc.robotcontroller.Math.Vectors;


//import org.jetbrains.annotations.NotNull;

import java.util.Arrays;

public class Vector {
	private final double[] vec;
	private final int size;

	/**
	 *
	 * @param vec vector
	 */
	public Vector(double [] vec) {
		this.size = vec.length;
		this.vec = vec;
	}

	public Vector(int size) {
		this.size = size;
		this.vec = new double[size];
		initializeToZero();
	}

	/**
	 * set the value of an array
	 * @param value the value we want to insert
	 * @param index index we are putting the value into
	 */
	public void set(double value, int index) {
		vec[index] = value;
	}


	public double get(int index) {
		return vec[index];
	}

	private void initializeToZero() {
		Arrays.fill(vec, 0);
	}


	public Vector add(Vector other) throws Exception {
		checkVecCompatibility(other);
		Vector nu = new Vector(this.size);
		for (int i = 0; i < this.size; ++i) {
			nu.set(this.vec[i] + other.vec[i],i);
		}
		return nu;
	}

	public Vector minus(Vector other) throws Exception {
		return this.add(other.negate());
	}

	/**
	 * Compute the dot product of two vectors
	 * @param other the other vector
	 * @return the dot product
	 * @throws Exception raise if vectors are not equal length
	 */
	public double dotProduct(Vector other) throws Exception {
		checkVecCompatibility(other);
		double sum = 0;
		for (int i = 0; i < vec.length; ++i) {
			sum += other.vec[i] * this.vec[i];
		}
		return sum;
	}

	public Vector crossProduct(Vector other) throws Exception{
		checkVecCompatibility(other);
		if (vec.length != 3 && vec.length != 2){
			throw new Exception("Vector is not 3D and does not have a cross product");
		}
		return new Vector(0);
	}
	public Vector scalarMultiply(double scalar) {
		Vector nu = new Vector(this.vec.length);
		for (int i = 0; i < this.vec.length; ++i) {
			nu.set(this.vec[i] * scalar,i);
		}
		return nu;
	}

	public Vector negate() {
		return scalarMultiply(-1);
	}

	public Vector clone() {
		Vector nu = new Vector(this.size);
		for (int i = 0; i < this.vec.length; ++i) {
			nu.set(this.vec[i],i);
		}
		return nu;
	}

	private void checkVecCompatibility(Vector other) throws Exception {
		if (other.vec.length != this.vec.length) {
			throw new Exception("Vector length (" +vec.length + ") is not equal to " + other.vec.length);
		}
	}
}
