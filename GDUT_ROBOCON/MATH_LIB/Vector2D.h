#ifndef VECTOR2D_H
#define VECTOR2D_H

#ifdef __cplusplus
extern "C"
{
#endif
#include <arm_math.h>

#ifdef __cplusplus
}
#endif
#ifdef __cplusplus
class Vector2D
{
public:
    float32_t x = 0.0f;
    float32_t y = 0.0f;

    Vector2D();
    Vector2D(float32_t x_, float32_t y_);

    Vector2D &operator=(const Vector2D &other);

    Vector2D operator+(const Vector2D &other) const;

    Vector2D operator-(const Vector2D &other) const;

    // 使用DSP库实现向量点乘
    float32_t operator*(const Vector2D &other) const;

    // 向量乘以标量
    Vector2D operator*(float32_t scalar) const;

    // 向量的模（长度）
    float32_t magnitude() const;

    // 单位化向量
    Vector2D normalize() const;

    // 向量投影：投影this向量到other向量上
    Vector2D project_onto(const Vector2D &other) const;
};

#endif // VECTOR2D_H
#endif