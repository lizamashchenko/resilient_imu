#ifndef MATRIX_H
#define MATRIX_H

#include <vector>
#include <stdexcept>
#include "Vector.h"

template <typename T>
class Matrix {
    std::vector<std::vector<T>> data;
    size_t rows;
    size_t cols;

public:
    Matrix() : rows(0), cols(0) {}

    Matrix(size_t r, size_t c) : rows(r), cols(c), data(r, std::vector<T>(c, T())) {}
    Matrix(size_t r, size_t c, const std::vector<std::vector<T>>& values)
    : rows(r), cols(c), data(values)
    {
        if (values.size() != r)
            throw std::invalid_argument("Row count does not match provided data");

        for (const auto& row : values) {
            if (row.size() != c)
                throw std::invalid_argument("Column count mismatch in provided data");
        }
    }

    Matrix(const Matrix<T>& other) = default;
    Matrix(Matrix<T>&& other) = default;
    Matrix<T>& operator=(const Matrix<T>& other) = default;
    Matrix<T>& operator=(Matrix<T>&& other) = default;

    size_t row_count() const { return rows; }
    size_t col_count() const { return cols; }

    Matrix transpose() const {
        if (data.empty()) return Matrix<T>();

        size_t rows = data.size();
        size_t cols = data[0].size();
        Matrix<T> result;

        for (size_t col = 0; col < cols; ++col) {
            std::vector<T> new_row;
            for (size_t row = 0; row < rows; ++row) {
                new_row.push_back(data[row][col]);
            }
            result.add_row(new_row);
        }

        return result;
    }

    void add_row(const std::vector<T>& row) {
        if (cols == 0) {
            cols = row.size();  // first row determines column count
        } else if (row.size() != cols) {
            throw std::invalid_argument("All rows must have the same number of columns");
        }
        data.push_back(row);
        rows++;
    }

    Matrix<T> operator+(const Matrix<T>& other) const {
        if (rows != other.rows || cols != other.cols)
            throw std::invalid_argument("Matrix dimensions must match for addition");

        Matrix<T> result(rows, cols);
        for (size_t i = 0; i < rows; ++i)
            for (size_t j = 0; j < cols; ++j)
                result.data[i][j] = data[i][j] + other.data[i][j];
        return result;
    }

    Matrix<T> operator-(const Matrix<T>& other) const {
        if (rows != other.rows || cols != other.cols)
            throw std::invalid_argument("Matrix dimensions must match for subtraction");

        Matrix<T> result(rows, cols);
        for (size_t i = 0; i < rows; ++i)
            for (size_t j = 0; j < cols; ++j)
                result.data[i][j] = data[i][j] - other.data[i][j];
        return result;
    }

    Matrix<T> operator*(const Matrix<T>& other) const {
        if (cols != other.rows)
            throw std::invalid_argument("Invalid dimensions for matrix multiplication");

        Matrix<T> result(rows, other.cols);
        for (size_t i = 0; i < rows; ++i)
            for (size_t j = 0; j < other.cols; ++j)
                for (size_t k = 0; k < cols; ++k)
                    result.data[i][j] += data[i][k] * other.data[k][j];
        return result;
    }

    Vector<T> operator*(const Vector<T>& vec) const {
        if (vec.size() != cols)
            throw std::invalid_argument("Vector size must match number of matrix columns");

        Vector<T> result(rows);
        for (size_t i = 0; i < rows; ++i)
            for (size_t j = 0; j < cols; ++j)
                result[i] += data[i][j] * vec[j];
        return result;
    }

    const std::vector<T>& operator[](size_t i) const { return data[i]; }
    std::vector<T>& operator[](size_t i) { return data[i]; }

    void set(size_t i, size_t j, const T& value) {
        if (i >= rows || j >= cols)
            throw std::out_of_range("Matrix index out of bounds");
        data[i][j] = value;
    }

    T get(size_t i, size_t j) const {
        if (i >= rows || j >= cols)
            throw std::out_of_range("Matrix index out of bounds");
        return data[i][j];
    }
};

#endif // MATRIX_H
