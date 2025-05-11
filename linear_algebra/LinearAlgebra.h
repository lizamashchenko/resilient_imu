#ifndef LINEAR_ALGEBRA_H
#define LINEAR_ALGEBRA_H

#include <vector>
#include <stdexcept>
#include <cmath>

namespace linear_algebra {
    template <typename T>
    class Matrix;

    template <typename T>
    class Vector {
    private:
        std::vector<T> data;

    public:
        Vector() = default;

        explicit Vector(const std::vector<T>& data) : data(data) {}
        explicit Vector(size_t size) : data(size) {}
        Vector(std::initializer_list<T> list) : data(list) {}

        Vector(const Vector& other) = default;
        Vector(Vector&& other) noexcept = default;
        Vector& operator=(const Vector& other) = default;
        Vector& operator=(Vector&& other) noexcept = default;

        ~Vector() = default;

        size_t size() const {
            return data.size();
        }

        void normalize() {
            T norm = T();
            for (const auto& val : data) {
                norm += val * val;
            }
            norm = std::sqrt(norm);
            if (norm == 0) return;
            for (auto& val : data) {
                val /= norm;
            }
        }

        T& operator[](size_t i) {
            return data[i];
        }

        const T& operator[](size_t i) const {
            return data[i];
        }

        Vector operator+(const Vector& other) const {
            if (data.size() != other.data.size())
                throw std::invalid_argument("Vectors must be the same size for addition");

            Vector result;
            result.data.resize(data.size());
            for (size_t i = 0; i < data.size(); ++i) {
                result.data[i] = data[i] + other.data[i];
            }
            return result;
        }

        Vector operator-(const Vector& other) const {
            if (data.size() != other.data.size())
                throw std::invalid_argument("Vectors must be the same size for subtraction");

            Vector result;
            result.data.resize(data.size());
            for (size_t i = 0; i < data.size(); ++i) {
                result.data[i] = data[i] - other.data[i];
            }
            return result;
        }

        Vector operator*(const Vector& other) const {
            if (data.size() != other.data.size())
                throw std::invalid_argument("Vectors must be the same size for element-wise multiplication");

            Vector result;
            result.data.resize(data.size());
            for (size_t i = 0; i < data.size(); ++i) {
                result.data[i] = data[i] * other.data[i];
            }
            return result;
        }
        Vector<T> operator*(const Matrix<T>& matrix) const {
            if (this->size() != matrix.row_count())
                throw std::invalid_argument("Vector size must match number of matrix rows for multiplication");

            Vector<T> result(matrix.col_count());
            for (size_t j = 0; j < matrix.col_count(); ++j) {
                for (size_t i = 0; i < this->size(); ++i) {
                    result[j] += (*this)[i] * matrix[i][j];
                }
            }
            return result;
        }
        Vector operator*(double scale) const {
            Vector result;
            result.data.resize(data.size());
            for (size_t i = 0; i < data.size(); ++i) {
                result.data[i] = data[i] * scale;
            }
            return result;
        }

        void push_back(const T& value) {
            data.push_back(value);
        }

        typename std::vector<T>::iterator begin() { return data.begin(); }
        typename std::vector<T>::iterator end() { return data.end(); }
        typename std::vector<T>::const_iterator begin() const { return data.begin(); }
        typename std::vector<T>::const_iterator end() const { return data.end(); }

        Matrix<T> transpose_as_row() const {
            Matrix<T> result(1, this->size());
            for (size_t i = 0; i < this->size(); ++i)
                result[0][i] = (*this)[i];
            return result;
        }

        Matrix<T> transpose_as_column() const {
            Matrix<T> result(this->size(), 1);
            for (size_t i = 0; i < this->size(); ++i)
                result[i][0] = (*this)[i];
            return result;
        }

    };

    template <typename T>
    class Matrix {
        std::vector<std::vector<T>> data;
        size_t rows;
        size_t cols;

    public:
        Matrix() : rows(0), cols(0) {}

        Matrix(size_t r, size_t c) : data(r, std::vector<T>(c, T())), rows(r), cols(c) {}
        Matrix(size_t r, size_t c, const std::vector<std::vector<T>>& values) : data(values), rows(r), cols(c) {
            if (values.size() != r)
                throw std::invalid_argument("Row count does not match provided data");

            for (const auto& row : values) {
                if (row.size() != c)
                    throw std::invalid_argument("Column count mismatch in provided data");
            }
        }

        Matrix(const Matrix& other) = default;
        Matrix(Matrix&& other) = default;
        Matrix& operator=(const Matrix& other) = default;
        Matrix& operator=(Matrix&& other) = default;

        size_t row_count() const { return rows; }
        size_t col_count() const { return cols; }

        Matrix transpose() const {
            if (data.empty()) return Matrix();

            size_t rows = data.size();
            size_t cols = data[0].size();
            Matrix result;

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
                cols = row.size();
            } else if (row.size() != cols) {
                throw std::invalid_argument("All rows must have the same number of columns");
            }
            data.push_back(row);
            rows++;
        }

        Matrix operator+(const Matrix& other) const {
            if (rows != other.rows || cols != other.cols)
                throw std::invalid_argument("Matrix dimensions must match for addition");

            Matrix result(rows, cols);
            for (size_t i = 0; i < rows; ++i)
                for (size_t j = 0; j < cols; ++j)
                    result.data[i][j] = data[i][j] + other.data[i][j];
            return result;
        }

        Matrix operator-(const Matrix<T>& other) const {
            if (rows != other.rows || cols != other.cols)
                throw std::invalid_argument("Matrix dimensions must match for subtraction");

            Matrix result(rows, cols);
            for (size_t i = 0; i < rows; ++i)
                for (size_t j = 0; j < cols; ++j)
                    result.data[i][j] = data[i][j] - other.data[i][j];
            return result;
        }

        Matrix operator*(const Matrix& other) const {
            if (cols != other.rows)
                throw std::invalid_argument("Invalid dimensions for matrix multiplication");

            Matrix result(rows, other.cols);
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

        Matrix operator/(double denom) const {
            Matrix result(rows, cols);

            for (size_t i = 0; i < rows; ++i) {
                for (size_t j = 0; j < cols; ++j) {
                    result.data[i][j] = this->data[i][j] / denom;
                }
            }

            return result;
        }

        Matrix operator*(double scale) const {
            Matrix result(rows, cols);

            for (size_t i = 0; i < rows; ++i) {
                for (size_t j = 0; j < cols; ++j) {
                    result.data[i][j] = this->data[i][j] * scale;
                }
            }

            return result;
        }

    };
}
#endif //LINEAR_ALGEBRA_H
