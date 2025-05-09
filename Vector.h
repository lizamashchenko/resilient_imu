#ifndef VECTOR_H
#define VECTOR_H

#include <vector>
#include <stdexcept>
#include <cmath>

template <typename T>
class Vector {
private:
    std::vector<T> data;

public:
    Vector() = default;

    explicit Vector(const std::vector<T>& data) : data(data) {}
    explicit Vector(size_t rows) : data(rows) {}

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

    Vector operator*(double coeff) const {
        Vector result;
        result.data.resize(data.size());
        for (size_t i = 0; i < data.size(); ++i) {
            result.data[i] = data[i] * coeff;
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
};

#endif // VECTOR_H
