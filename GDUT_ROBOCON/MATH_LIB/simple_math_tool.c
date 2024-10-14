#include "simple_math_tool.h"

float get_distan(float x1, float y1, float x2, float y2)
{
    float distance = (x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1);
    return sqrt(distance);
}