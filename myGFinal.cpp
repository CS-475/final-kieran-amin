#include "myGFinal.h"
#include <cmath>


namespace {
    std::shared_ptr<GPath> final_addStrokedLine(GPoint p0, GPoint p1, float width, bool roundCap) {
        float dx = p1.x - p0.x;
        float dy = p1.y - p0.y;
        float length = sqrt(dx*dx + dy*dy);
        
        if (length < 0.0001f) return nullptr;
        
        float nx = (-dy / length) * (width/2);
        float ny = (dx / length) * (width/2);
        
        GPoint point0 = {p0.x + nx, p0.y + ny};
        GPoint point1 = {p0.x - nx, p0.y - ny};
        GPoint point2 = {p1.x - nx, p1.y - ny};
        GPoint point3 = {p1.x + nx, p1.y + ny};

        GPathBuilder bobTheBuilder;
        bobTheBuilder.moveTo(point0);
        bobTheBuilder.lineTo(point1);
        bobTheBuilder.lineTo(point2);
        bobTheBuilder.lineTo(point3);

        return bobTheBuilder.detach();
    }
}


std::shared_ptr<GShader> MyGFinal::createLinearPosGradient(GPoint p0, GPoint p1, const GColor colors[], const float pos[], int count) {
    if (count < 1 || !colors || !pos) {
        return nullptr;
    }

    std::vector<GColor> interpolatedColors;
    

    for (float t = 0; t <= 1.0f; t += 0.01f) {  

        int index = 0;
        while (index < count - 1 && t > pos[index + 1]) {
            index++;
        }

        GColor currentColor;
        if (t <= pos[0]) {
            currentColor = colors[0];
        } else if (t >= pos[count - 1]) {
            currentColor = colors[count - 1];
        } else {
            // Interpolate between colors
            float t0 = pos[index];
            float t1 = pos[index + 1];
            float weight = (t - t0) / (t1 - t0);

            const GColor& c0 = colors[index];
            const GColor& c1 = colors[index + 1];

            currentColor = {
                c0.r + (c1.r - c0.r) * weight,
                c0.g + (c1.g - c0.g) * weight,
                c0.b + (c1.b - c0.b) * weight,
                c0.a + (c1.a - c0.a) * weight
            };
        }
        interpolatedColors.push_back(currentColor);
    }

    return GCreateLinearGradient(p0, p1, interpolatedColors.data(), interpolatedColors.size(), GTileMode::kClamp);
}

std::shared_ptr<GShader> MyGFinal::createVoronoiShader(const GPoint points[], const GColor colors[], int count) {
    return nullptr;
}

// uses strokeLine helper
std::shared_ptr<GPath> MyGFinal::strokePolygon(const GPoint points[], int count, float width, bool isClosed) {
    if (count < 2) return nullptr;
    
    // For count = 2, return the stroked line
    if (count == 2) {
        return final_addStrokedLine(points[0], points[1], width, false);
    }   
    
// special case for 3 points?
    
    GPathBuilder bobTheBuilder;
    float halfWidth = width/2;

    auto getNormal = [](GPoint p0, GPoint p1, float scale) -> std::pair<float, float> {
        float dx = p1.x - p0.x;
        float dy = p1.y - p0.y;
        float length = sqrt(dx*dx + dy*dy);
        if (length < 0.0001f) {
            return {0, scale};
        }
        return {(-dy / length) * scale, (dx / length) * scale};
    };

    auto addRoundedJoint = [&bobTheBuilder, halfWidth](GPoint center, float nx1, float ny1, float nx2, float ny2) {
        float angle1 = atan2(ny1, nx1);
        float angle2 = atan2(ny2, nx2);
        
        // for identical angles
        if (fabs(angle1 - angle2) < 0.0001f) {
            // angles are the same so just draw a line
            bobTheBuilder.lineTo({center.x + nx2, center.y + ny2});
            return;
        }
        
        // shortest path around circle
        if (angle2 - angle1 > M_PI) angle2 -= 2 * M_PI;
        if (angle1 - angle2 > M_PI) angle1 -= 2 * M_PI;
        
        float angleDiff = fabs(angle2 - angle1);
        int stepCount;
        
        if (angleDiff > M_PI * 0.75) {
            stepCount = 12;
        } else if (angleDiff > M_PI * 0.5) {
            stepCount = 8;
        } else {
            stepCount = 6;
        }

        for (int i = 0; i <= stepCount; i++) {
            float t = float(i) / stepCount;
            float angle = angle1 + (angle2 - angle1) * t;
            
            float x = center.x + cos(angle) * halfWidth;
            float y = center.y + sin(angle) * halfWidth;
            
            // Add the point to the path
            bobTheBuilder.lineTo({x, y});
        }
    };

    auto [nx0, ny0] = getNormal(points[0], points[1], halfWidth);

    bobTheBuilder.moveTo({points[0].x + nx0, points[0].y + ny0});

    // outer path
    for (int i = 1; i < count; i++) {
        auto [nx_curr, ny_curr] = getNormal(points[i-1], points[i], halfWidth);
        
        if (i < count-1) {
            auto [nx_next, ny_next] = getNormal(points[i], points[i+1], halfWidth);
            addRoundedJoint(points[i], nx_curr, ny_curr, nx_next, ny_next);
        } else {
            if (!isClosed) {
                auto line = final_addStrokedLine(points[i-1], points[i], width, false);
                if (line) {
                    bobTheBuilder.lineTo({points[i].x + nx_curr, points[i].y + ny_curr});
                }
            } else {
                bobTheBuilder.lineTo({points[i].x + nx_curr, points[i].y + ny_curr});
            }
        }
    }

    // If closed, add rounded joint
    if (isClosed) {
        auto [normalX_end, normalY_end] = getNormal(points[count-1], points[0], halfWidth);
        auto [normalX_start, normalY_start] = getNormal(points[0], points[1], halfWidth);
        addRoundedJoint(points[0], normalX_end, normalY_end, normalX_start, normalY_start);
        bobTheBuilder.lineTo({points[0].x + normalX_start, points[0].y + normalY_start});
    } else {
        auto [nX_end, nY_end] = getNormal(points[count-2], points[count-1], halfWidth);
        bobTheBuilder.lineTo({points[count-1].x + nX_end, points[count-1].y + nY_end});
    }

    for (int i = count-1; i >= 0; i--) {
        auto [nx_curr, ny_curr] = getNormal(points[i], points[(i + 1) % count], halfWidth);
        
        if (i > 0) {
            auto [nx_prev, ny_prev] = getNormal(points[i-1], points[i], halfWidth);
            addRoundedJoint(points[i], -nx_curr, -ny_curr, -nx_prev, -ny_prev);
        } else {
            bobTheBuilder.lineTo({points[i].x - nx_curr, points[i].y - ny_curr});
        }
    }

    bobTheBuilder.lineTo({points[0].x + nx0, points[0].y + ny0});

    return bobTheBuilder.detach();
}

// from board
std::unique_ptr<GFinal> GCreateFinal() {
    return std::make_unique<MyGFinal>();
}