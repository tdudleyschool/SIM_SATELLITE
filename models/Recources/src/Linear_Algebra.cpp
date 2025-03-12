#ifndef LINEAR_ALGEBRA
#define LINEAR_ALGEBRA


#include <cmath>
#include <iostream>
#include <cmath>
#include <stdexcept>
#include <array>

#define ORANGE "\033[38;5;208m" //orange color
#define RESET "\033[0m"


using namespace std;


//=========================================
class Vector3d {
private:
    int insert_index = 0;

public:
    double x, y, z;
    // Constructor to initialize the vector
    Vector3d(double x = 0.0, double y = 0.0, double z = 0.0) : x(x), y(y), z(z) {}

    // Overload the << operator for setting vector elements
    Vector3d& operator<<(const Vector3d& other) {
        x = other.x;
        y = other.y;
        z = other.z;
        return *this;
    }

    // Function to insert values for x, y, z
    void insert(double x_val, double y_val, double z_val) {
        x = x_val;
        y = y_val;
        z = z_val;
    }

    // Overload the = operator for assignment
    Vector3d& operator=(const Vector3d& other) {
        if (this != &other) {
            x = other.x;
            y = other.y;
            z = other.z;
        }
        return *this;
    }

    // Overload the + operator for vector addition
    Vector3d operator+(const Vector3d& other) const {
        return Vector3d(x + other.x, y + other.y, z + other.z);
    }

    // Overload the - operator for vector subtraction
    Vector3d operator-(const Vector3d& other) const {
        return Vector3d(x - other.x, y - other.y, z - other.z);
    }

    // Overload the * operator for vector-scalar multiplication
    Vector3d operator*(double scalar) const {
        return Vector3d(x * scalar, y * scalar, z * scalar);
    }

    friend Vector3d operator*(double scalar, const Vector3d& vec) {
        return Vector3d(scalar*vec.x, scalar*vec.y, scalar*vec.z);
    }

    // Overload the / operator for vector-scalar division
    Vector3d operator/(double scalar) const {
        if (scalar != 0) {
            return Vector3d(x / scalar, y / scalar, z / scalar);
        } else {
            //return Vector3d(x / 1, y / 1, z / 1);
            throw std::invalid_argument("Division by zero in file " + std::string(__FILE__) + "at line " + std::to_string(__LINE__) + "!!!");
        }
    }

    // Overload the * operator for dot product
    double dot(const Vector3d& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    // Cross product
    Vector3d cross(const Vector3d& other) const {
        return Vector3d(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    // Magnitude of the vector (Norm)
    double norm() const {
        return std::sqrt(x * x + y * y + z * z);
    }

    // Alternative name for norm() -> magnitude
    double mag() const {
        return norm();
    }

    // Normalize the vector (make the magnitude 1)
    void normalize() {
        double len = norm();
        if (len != 0) {
            x /= len;
            y /= len;
            z /= len;
        }
    }

    Vector3d normalized() {
        double len = norm();
        if (len != 0){
            return Vector3d(x/len, y/len, z/len);
        }
        else
            cerr << ORANGE << "Length Is Zero!!!";
            return Vector3d(1, 0, 0);
    }
//Division by zero
    // Overload the [] operator for element access
    double& operator[](int index) {
        if (index == 0) return x;
        else if (index == 1) return y;
        else if (index == 2) return z;
        else throw std::out_of_range("Index out of range");
    }

    // Const version of the [] operator for element access
    const double& operator[](int index) const {
        if (index == 0) return x;
        else if (index == 1) return y;
        else if (index == 2) return z;
        else throw std::out_of_range("Index out of range");
    }

    // Access using () operator
    double operator()(int index) const {
        if (index == 0) return x;
        else if (index == 1) return y;
        else if (index == 2) return z;
        else throw std::out_of_range("Index out of range");
    }

    // Output the vector using the << operator
    friend std::ostream& operator<<(std::ostream& os, const Vector3d& vec) {
        os << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
        return os;
    }
};

class Matrix3d {
private:
    double mat[3][3];
    int insert_index = 0; // Tracks which element to insert next

public:
    // Constructor to initialize the matrix (default to zero)
    Matrix3d(double val = 0.0) {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                mat[i][j] = val;
    }

    // Function to insert values for a 3x3 matrix
    void insert(double m11, double m12, double m13, 
                double m21, double m22, double m23, 
                double m31, double m32, double m33) {
        mat[0][0] = m11; mat[0][1] = m12; mat[0][2] = m13;
        mat[1][0] = m21; mat[1][1] = m22; mat[1][2] = m23;
        mat[2][0] = m31; mat[2][1] = m32; mat[2][2] = m33;
    }

    // Overload the << operator for copying another matrix
    Matrix3d& operator<<(const Matrix3d& other) {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                mat[i][j] = other.mat[i][j];
        return *this;
    }

    // Overload the = operator for matrix assignment
    Matrix3d& operator=(const Matrix3d& other) {
        if (this != &other) {
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j)
                    mat[i][j] = other.mat[i][j];
        }
        return *this;
    }

    // Matrix addition
    Matrix3d operator+(const Matrix3d& other) const {
        Matrix3d result;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                result.mat[i][j] = mat[i][j] + other.mat[i][j];
        return result;
    }

    // Matrix subtraction
    Matrix3d operator-(const Matrix3d& other) const {
        Matrix3d result;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                result.mat[i][j] = mat[i][j] - other.mat[i][j];
        return result;
    }

    // Matrix multiplication with another matrix
    Matrix3d operator*(const Matrix3d& other) const {
        Matrix3d result;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                result.mat[i][j] = 0;
                for (int k = 0; k < 3; ++k)
                    result.mat[i][j] += mat[i][k] * other.mat[k][j];
            }
        return result;
    }

    // Matrix-vector multiplication (Vector3d * Matrix3d)
    Vector3d operator*(const Vector3d& vec) const {
        double x = mat[0][0] * vec.x + mat[0][1] * vec.y + mat[0][2] * vec.z;
        double y = mat[1][0] * vec.x + mat[1][1] * vec.y + mat[1][2] * vec.z;
        double z = mat[2][0] * vec.x + mat[2][1] * vec.y + mat[2][2] * vec.z;
        return Vector3d(x, y, z);
    }

    // Matrix transpose
    Matrix3d transpose() const {
        Matrix3d result;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                result.mat[i][j] = mat[j][i];
        return result;
    }

    // Matrix inverse (for 3x3 matrix)
    Matrix3d inverse() const {
        Matrix3d result;
        double det = determinant();
        if (det == 0)
            throw std::invalid_argument("Matrix is singular and cannot be inverted.");

        result.mat[0][0] = (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1]) / det;
        result.mat[0][1] = (mat[0][2] * mat[2][1] - mat[0][1] * mat[2][2]) / det;
        result.mat[0][2] = (mat[0][1] * mat[1][2] - mat[0][2] * mat[1][1]) / det;
        result.mat[1][0] = (mat[1][2] * mat[2][0] - mat[1][0] * mat[2][2]) / det;
        result.mat[1][1] = (mat[0][0] * mat[2][2] - mat[0][2] * mat[2][0]) / det;
        result.mat[1][2] = (mat[0][2] * mat[1][0] - mat[0][0] * mat[1][2]) / det;
        result.mat[2][0] = (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]) / det;
        result.mat[2][1] = (mat[0][1] * mat[2][0] - mat[0][0] * mat[2][1]) / det;
        result.mat[2][2] = (mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0]) / det;

        return result;
    }

    // Get the determinant of the matrix
    double determinant() const {
        return mat[0][0] * (mat[1][1] * mat[2][2] - mat[1][2] * mat[2][1]) -
               mat[0][1] * (mat[1][0] * mat[2][2] - mat[1][2] * mat[2][0]) +
               mat[0][2] * (mat[1][0] * mat[2][1] - mat[1][1] * mat[2][0]);
    }

    // Extract a specific column
    Vector3d col(int index) const {
        if (index < 0 || index >= 3)
            throw std::out_of_range("Index out of range");
        return Vector3d(mat[0][index], mat[1][index], mat[2][index]);
    }

    // Access an element of the matrix
    double operator()(int i, int j) const {
        if (i < 0 || i >= 3 || j < 0 || j >= 3)
            throw std::out_of_range("Index out of range");
        return mat[i][j];
    }

    // Output the matrix using the << operator
    friend std::ostream& operator<<(std::ostream& os, const Matrix3d& mat) {
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                os << mat.mat[i][j] << (j < 2 ? ", " : "");
            }
            os << (i < 2 ? "\n" : "");
        }
        return os;
    }
};

class Quaterniond {
    
public:
    // Default constructor
    double _w, _x, _y, _z;

    Quaterniond() : _w(1), _x(0), _y(0), _z(0) {}

    // Parameterized constructor to set values
    Quaterniond(double w, double x, double y, double z) : _w(w), _x(x), _y(y), _z(z) {}

    // Set values together
    void set(double w_val, double x_val, double y_val, double z_val) {
        _w = w_val;
        _x = x_val;
        _y = y_val;
        _z = z_val;
    }

    // Getter functions (const because we don't modify them)
    double w() const { return _w; }
    double x() const { return _x; }
    double y() const { return _y; }
    double z() const { return _z; }

    // Setter functions (non-const to allow modification)
    void w(double w_val) { _w = w_val; }
    void x(double x_val) { _x = x_val; }
    void y(double y_val) { _y = y_val; }
    void z(double z_val) { _z = z_val; }

    // Normalize the quaternion
    void normalize() {
        double norm = std::sqrt(_w*_w + _x*_x + _y*_y + _z*_z);
        if (norm > 0.0) {
            _w /= norm;
            _x /= norm;
            _y /= norm;
            _z /= norm;
        } else {
            throw std::invalid_argument("Cannot normalize a zero quaternion");
        }
    }

    // Convert quaternion to rotation matrix
    Matrix3d toRotationMatrix() const {
        Matrix3d mat;

        mat.insert(1 - 2*(_y*_y + _z*_z), 2*(_x*_y - _w*_z), 2*(_x*_z + _w*_y),
              2*(_x*_y + _w*_z), 1 - 2*(_x*_x + _z*_z), 2*(_y*_z - _w*_x),
              2*(_x*_z - _w*_y), 2*(_y*_z + _w*_x), 1 - 2*(_x*_x + _y*_y));

        return mat;
    }

    // Overload + operator (quaternion addition)
    Quaterniond operator+(const Quaterniond& other) const {
        return Quaterniond(_w + other._w, _x + other._x, _y + other._y, _z + other._z);
    }

    // Overload - operator (quaternion subtraction)
    Quaterniond operator-(const Quaterniond& other) const {
        return Quaterniond(_w - other._w, _x - other._x, _y - other._y, _z - other._z);
    }

    // Overload * operator (quaternion multiplication)
    Quaterniond operator*(const Quaterniond& other) const {
        return Quaterniond(
            _w*other._w - _x*other._x - _y*other._y - _z*other._z,
            _w*other._x + _x*other._w + _y*other._z - _z*other._y,
            _w*other._y - _x*other._z + _y*other._w + _z*other._x,
            _w*other._z + _x*other._y - _y*other._x + _z*other._w
        );
    }

    // Overload / operator (quaternion division, by scalar)
    Quaterniond operator/(double scalar) const {
        if (scalar == 0) {
            throw std::invalid_argument("Cannot divide by zero");
        }
        return Quaterniond(_w / scalar, _x / scalar, _y / scalar, _z / scalar);
    }

    // Print the quaternion for debugging
    void print() const {
        std::cout << "[" << _w << ", " << _x << ", " << _y << ", " << _z << "]" << std::endl;
    }
};

class Vector2d {
private:
    std::array<double, 2> values;

public:
    // Default constructor
    Vector2d() : values{0, 0} {}

    // Insert method
    void insert(double x, double y) {
        values[0] = x;
        values[1] = y;
    }

    // Overload [] operator for element access
    double& operator[](size_t index) {
        if (index >= 2) throw std::out_of_range("Index out of range");
        return values[index];
    }

    const double& operator[](size_t index) const {
        if (index >= 2) throw std::out_of_range("Index out of range");
        return values[index];
    }

    // Print method for debugging
    void print() const {
        std::cout << "(" << values[0] << ", " << values[1] << ")" << std::endl;
    }
};

class Matrix2d {
private:
    std::array<std::array<double, 2>, 2> values;

public:
    // Default constructor
    Matrix2d() : values{{{0, 0}, {0, 0}}} {}

    // Insert method
    void insert(double a11, double a12, double a21, double a22) {
        values[0][0] = a11;
        values[0][1] = a12;
        values[1][0] = a21;
        values[1][1] = a22;
    }

    // Overload * operator for Matrix2D * Vector2D
    Vector2d operator*(const Vector2d& vec) const {
        Vector2d result;
        result.insert(
            values[0][0] * vec[0] + values[0][1] * vec[1],
            values[1][0] * vec[0] + values[1][1] * vec[1]
        );
        return result;
    }

    // Overload * operator for Vector2D * Matrix2D
    friend Vector2d operator*(const Vector2d& vec, const Matrix2d& mat) {
        Vector2d result;
        result.insert(
            vec[0] * mat.values[0][0] + vec[1] * mat.values[1][0],
            vec[0] * mat.values[0][1] + vec[1] * mat.values[1][1]
        );
        return result;
    }

    // Print method for debugging
    void print() const {
        std::cout << "[" << values[0][0] << ", " << values[0][1] << "]" << std::endl;
        std::cout << "[" << values[1][0] << ", " << values[1][1] << "]" << std::endl;
    }
};


#endif
