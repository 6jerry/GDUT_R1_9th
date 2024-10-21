#include "Vector2D.h"


Vector2D::Vector2D() : x(0), y(0) {}
Vector2D::Vector2D(float32_t x_, float32_t y_) : x(x_), y(y_) {}

// 重载赋值运算符
Vector2D &Vector2D::operator=(const Vector2D &other)
{
    if (this != &other)
    {
        this->x = other.x;
        this->y = other.y;
    }
    return *this;
}

// 向量加法
Vector2D Vector2D::operator+(const Vector2D &other) const
{
    return Vector2D(this->x + other.x, this->y + other.y);
}

// 向量减法
Vector2D Vector2D::operator-(const Vector2D &other) const
{
    return Vector2D(this->x - other.x, this->y - other.y);
}

// 使用DSP库实现向量点乘
float32_t Vector2D::operator*(const Vector2D &other) const
{
    float32_t dotProduct;
    float32_t vecA[2] = {this->x, this->y};
    float32_t vecB[2] = {other.x, other.y};
    arm_dot_prod_f32(vecA, vecB, 2, &dotProduct);
    return dotProduct;
}

// 向量乘以标量
Vector2D Vector2D::operator*(float32_t scalar) const
{
    return Vector2D(this->x * scalar, this->y * scalar);
}

// 向量的模
float32_t Vector2D::magnitude() const
{
    float32_t result;
    arm_sqrt_f32((this->x * this->x) + (this->y * this->y), &result);
    return result;
}

// 单位化向量
Vector2D Vector2D::normalize() const
{
    Vector2D result;
    float32_t mag = this->magnitude();
    if (mag > 0)
    {
        result.x = this->x / mag;
        result.y = this->y / mag;
    }
    return result;
}

// 向量投影：投影this向量到other向量上
Vector2D Vector2D::project_onto(const Vector2D &other) const
{
    float32_t dotProduct = (*this) * other;
    float32_t magOtherSquared = other.x * other.x + other.y * other.y;
    float32_t scalar = dotProduct / magOtherSquared;
    return other * scalar;
}
