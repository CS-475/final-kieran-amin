#include "include/GCanvas.h"
#include "include/GPaint.h"
#include "include/GRect.h"
#include "include/GPixel.h"
#include "include/GMatrix.h"
#include "include/GPath.h"
#include "include/GPathBuilder.h"
#include "myGCanvas.h"
#include "blendUtils.h"
#include "edgeUtils.h"
#include "myGShader.h"

#include <iostream>

// Add at the top of the file with other utility functions
static inline int GDiv255(int value) {
    return (value + 128) * 257 >> 16;
}

// Add this declaration before the MyCanvas class
void drawTriangleInline(const GPoint verts[3], const GColor colors[3], const GPoint texs[3], 
                       const GPaint& paint, const GBitmap& device);

MyCanvas::MyCanvas(GBitmap& bitmap) : fBitmap(bitmap) {
}

MyCanvas::~MyCanvas() {
}

GPixel MyCanvas::colorToPixel(const GColor& color) const {
    uint8_t a = static_cast<uint8_t>(color.a * 255 + 0.5f);
    uint8_t r = static_cast<uint8_t>(color.r * color.a * 255 + 0.5f);
    uint8_t g = static_cast<uint8_t>(color.g * color.a * 255 + 0.5f);
    uint8_t b = static_cast<uint8_t>(color.b * color.a * 255 + 0.5f);
    return GPixel_PackARGB(a, r, g, b);
}

void MyCanvas::clear(const GColor& color) {
    int height = fBitmap.height();
    int width = fBitmap.width();
    GPixel p = colorToPixel(color);
    GPixel *row_addr = nullptr;
    for (int y = 0; y < height; y++) {
        row_addr = fBitmap.getAddr(0, y);
        for (int x = 0; x < width; x++) {
            row_addr[x] = p;
        }
    }
}


GPixel MyCanvas::blendPixel(GPixel dstPixel, GPixel srcPixel, GBlendMode blendMode) const {
    BlendProc blendFunc = findBlend(blendMode);
    return blendFunc(srcPixel, dstPixel);
}

void MyCanvas::drawRect(const GRect& rect, const GPaint& paint) {
    GPoint points[4] = {
        {rect.left, rect.top},     
        {rect.right, rect.top},    
        {rect.right, rect.bottom},  
        {rect.left, rect.bottom}    
    };

    drawConvexPolygon(points, 4, paint);

    // GPoint transformedPoints[4];
    // for (int i = 0; i < 4; ++i) {
    //     float sx = points[i].x;
    //     float sy = points[i].y;
    //     transformedPoints[i].x = fMatrix[0] * sx + fMatrix[2] * sy + fMatrix[4];
    //     transformedPoints[i].y = fMatrix[1] * sx + fMatrix[3] * sy + fMatrix[5];
    // }

    // GIRect ir = GIRect::LTRB(
    //     static_cast<int32_t>(std::min({transformedPoints[0].x, transformedPoints[1].x, transformedPoints[2].x, transformedPoints[3].x})),
    //     static_cast<int32_t>(std::min({transformedPoints[0].y, transformedPoints[1].y, transformedPoints[2].y, transformedPoints[3].y})),
    //     static_cast<int32_t>(std::max({transformedPoints[0].x, transformedPoints[1].x, transformedPoints[2].x, transformedPoints[3].x})),
    //     static_cast<int32_t>(std::max({transformedPoints[0].y, transformedPoints[1].y, transformedPoints[2].y, transformedPoints[3].y}))
    // );

    // int left = std::max(0, ir.left);
    // int top = std::max(0, ir.top);
    // int right = std::min(fBitmap.width(), ir.right);
    // int bottom = std::min(fBitmap.height(), ir.bottom);

    // if (left >= right || top >= bottom) {
    //     return; 
    // }

    // GShader* shader = paint.peekShader();
    // GPixel srcPixel;
    // bool useShader = (shader != nullptr && shader->setContext(fMatrix));

    // for (int y = top; y < bottom; ++y) {
    //     GPixel* rowAddr = fBitmap.getAddr(0, y);
    //     for (int x = left; x < right; ++x) {
    //         if (useShader) {
    //             shader->shadeRow(x, y, 1, &srcPixel);
    //         } else {
    //             srcPixel = colorToPixel(paint.getColor());
    //         }
    //         if (paint.getBlendMode() == GBlendMode::kSrc) {
    //             rowAddr[x] = srcPixel;
    //         } else {
    //             rowAddr[x] = blendPixel(rowAddr[x], srcPixel, paint.getBlendMode());
    //         }
    //     }
    // }
}


void MyCanvas::drawConvexPolygon(const GPoint points[], int count, const GPaint& paint) {
    GPoint transformedPoints[count];
    for (int i = 0; i < count; ++i) {
        float sx = points[i].x;
        float sy = points[i].y;
        transformedPoints[i].x = fMatrix[0] * sx + fMatrix[2] * sy + fMatrix[4];
        transformedPoints[i].y = fMatrix[1] * sx + fMatrix[3] * sy + fMatrix[5];
    }

    std::vector<Edge> edges;
    for (int i = 0; i < count; ++i) {
        int next = (i + 1) % count;
        Edge edge = makeEdge(transformedPoints[i], transformedPoints[next]);
        if (edge.isUseful()) {
            edges.push_back(edge);
        }
    }

    std::sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b) {
        return a.top < b.top;
    });

    int canvasHeight = fBitmap.height();
    GShader* shader = paint.peekShader();
    bool useShader = (shader != nullptr && shader->setContext(fMatrix));
    GPixel srcPixel;

    for (int y = 0; y < canvasHeight; ++y) {
        std::vector<int> intersections;

        for (const auto& edge : edges) {
            if (y >= edge.top && y <= edge.bottom) {
                int x = static_cast<int>(std::round(edge.m * y + edge.b));
                intersections.push_back(x);
            }
        }

        std::sort(intersections.begin(), intersections.end());

        if (useShader) {
            shader->shadeRow(0, y, 1, &srcPixel); 
        } else {
            srcPixel = colorToPixel(paint.getColor());
        }

        GPixel* rowAddr = fBitmap.getAddr(0, y);
        for (size_t i = 0; i < intersections.size(); i += 2) {
            if (i + 1 < intersections.size()) {
                int xStart = std::max(0, intersections[i]);
                int xEnd = std::min(fBitmap.width(), intersections[i + 1]);

                if (xStart < xEnd) {
                    for (int x = xStart; x < xEnd; ++x) {
                        if (useShader) {
                            shader->shadeRow(x, y, 1, &srcPixel);
                        }
                        rowAddr[x] = blendPixel(rowAddr[x], srcPixel, paint.getBlendMode());
                    }
                }
            }
        }
    }
}


void MyCanvas::save() {
    fMatrixStack.push(fMatrix);
}

void MyCanvas::restore() {
    if (!fMatrixStack.empty()) {
        fMatrix = fMatrixStack.top();
        fMatrixStack.pop();
    }
}

void MyCanvas::concat(const GMatrix& matrix) {
    fMatrix = GMatrix::Concat(fMatrix, matrix);
}

int computeQuadSegments(const GPoint pts[3], float tolerance) {
    float ax = pts[0].x - 2 * pts[1].x + pts[2].x;
    float ay = pts[0].y - 2 * pts[1].y + pts[2].y;
    float maxDist = std::sqrt(ax * ax + ay * ay);
    return static_cast<int>(std::ceil(std::sqrt(maxDist / tolerance)));

}

int computeCubicSegments(const GPoint pts[4], float tolerance) {
    float ax = -pts[0].x + 3 * (pts[1].x - pts[2].x) + pts[3].x;
    float ay = -pts[0].y + 3 * (pts[1].y - pts[2].y) + pts[3].y;
    float maxDist = std::sqrt(ax * ax + ay * ay);
    return static_cast<int>(std::ceil(std::sqrt(maxDist / tolerance)));
}

GPoint evalQuad(const GPoint pts[3], float t) {
    float u = 1 - t;
    return {
        u * u * pts[0].x + 2 * u * t * pts[1].x + t * t * pts[2].x,
        u * u * pts[0].y + 2 * u * t * pts[1].y + t * t * pts[2].y
    };
}

GPoint evalCubic(const GPoint pts[4], float t) {
    float u = 1 - t;
    return {
        u * u * u * pts[0].x + 3 * u * u * t * pts[1].x + 3 * u * t * t * pts[2].x + t * t * t * pts[3].x,
        u * u * u * pts[0].y + 3 * u * u * t * pts[1].y + 3 * u * t * t * pts[2].y + t * t * t * pts[3].y
    };
}

void MyCanvas::drawPath(const GPath& path, const GPaint& paint) {
    GPathBuilder builder;
    builder.reset();
    GPath::Iter iter(path);
    GPoint pts[GPath::kMaxNextPoints];
    const float tolerance = 0.25f;

    while (auto verb = iter.next(pts)) {
        switch (verb.value()) {
            case GPathVerb::kMove:
                builder.moveTo(pts[0]);
                break;
            case GPathVerb::kLine:
                builder.lineTo(pts[1]);
                break;
            case GPathVerb::kQuad: {
                int segments = computeQuadSegments(pts, tolerance);
                for (int i = 0; i < segments; ++i) {
                    float t1 = static_cast<float>(i + 1) / segments;
                    builder.lineTo(evalQuad(pts, t1));
                }
                break;
            }
            case GPathVerb::kCubic: {
                int segments = computeCubicSegments(pts, tolerance);
                for (int i = 0; i < segments; ++i) {
                    float t1 = static_cast<float>(i + 1) / segments;
                    builder.lineTo(evalCubic(pts, t1));
                }
                break;
            }
            default:
                break;
        }
    }

    builder.transform(fMatrix);
    auto transformedPath = builder.detach();
    
    GPath::Edger edger(*transformedPath);
    std::vector<Edge> edges;
    while (auto verb = edger.next(pts)) {
        if (verb.value() == GPathVerb::kLine) {
            Edge edge = makeEdge(pts[0], pts[1]);
            if (edge.isUseful()) {
                edges.push_back(edge);
            }
        }
    }
    
    std::sort(edges.begin(), edges.end(), [](const Edge& a, const Edge& b) {
        return a.top < b.top || (a.top == b.top && a.currX < b.currX);
    });

    int canvasHeight = fBitmap.height();
    int canvasWidth = fBitmap.width();
    GShader* shader = paint.peekShader();
    bool useShader = (shader != nullptr && shader->setContext(fMatrix));
    GPixel srcPixel;

    for (int y = 0; y < canvasHeight; ++y) {
        std::vector<int> xIntervals;
        int winding = 0;

        for (auto& edge : edges) {
            if (edge.isValid(y)) {
                int x = static_cast<int>(std::round(edge.computeX(y)));
                if (x < 0 || x >= canvasWidth) continue;
                xIntervals.push_back(x);
                winding += edge.winding;
            }
        }

        std::sort(xIntervals.begin(), xIntervals.end());

        for (size_t i = 0; i + 1 < xIntervals.size(); i += 2) {
            int left = xIntervals[i];
            int right = xIntervals[i + 1];
            left = std::max(0, left);
            right = std::min(canvasWidth, right);

            for (int xSpan = left; xSpan < right; ++xSpan) {
                if (useShader) {
                    shader->shadeRow(xSpan, y, 1, &srcPixel);
                } else {
                    srcPixel = colorToPixel(paint.getColor());
                }

                GPixel* dst = fBitmap.getAddr(xSpan, y);
                *dst = blendPixel(*dst, srcPixel, paint.getBlendMode());
            }
        }
    }
}


std::unique_ptr<GCanvas> GCreateCanvas(const GBitmap& bitmap) {
    return std::make_unique<MyCanvas>(const_cast<GBitmap&>(bitmap));
}


std::string GDrawSomething(GCanvas* canvas, GISize dim) {
    // Dark background
    canvas->clear(GColor::RGBA(0.15f, 0.15f, 0.2f, 1.0f));

    float canvasCenterX = dim.width / 2.0f;
    float canvasCenterY = dim.height / 2.0f;

    GBitmap bitmap;
    GMatrix localMatrix;
    std::shared_ptr<GShader> shader = GCreateBitmapShader(bitmap, localMatrix);

    // Create a spiral of triangles
    const int numTriangles = 8;
    const float radius = 120.0f;
    const float triangleSize = 40.0f;
    
    for (int i = 0; i < numTriangles; i++) {
        float angle = (i * 2 * M_PI) / numTriangles;
        float x = canvasCenterX + radius * cos(angle);
        float y = canvasCenterY + radius * sin(angle);
        float rotation = angle + M_PI / 3;  // Rotate each triangle

        GPoint triangle[] = {
            {x + triangleSize * cos(rotation), y + triangleSize * sin(rotation)},
            {x + triangleSize * cos(rotation + 2.0f * M_PI / 3), y + triangleSize * sin(rotation + 2.0f * M_PI / 3)},
            {x + triangleSize * cos(rotation + 4.0f * M_PI / 3), y + triangleSize * sin(rotation + 4.0f * M_PI / 3)}
        };
        
        // Create rainbow-like effect
        float hue = i / float(numTriangles);
        GPaint paint(GColor::RGBA(
            0.8f * (1.0f - hue),     // Red component
            0.8f * (0.5f + 0.5f * sin(hue * M_PI)),  // Green component
            0.8f * hue,              // Blue component
            0.85f));                 // Alpha
        paint.setShader(shader);
        canvas->drawConvexPolygon(triangle, 3, paint);
    }

    // Central star shape
    const int starPoints = 5;
    const float outerRadius = 80.0f;
    const float innerRadius = 30.0f;
    std::vector<GPoint> star;
    
    for (int i = 0; i < starPoints * 2; i++) {
        float angle = (i * M_PI) / starPoints;
        float r = (i % 2 == 0) ? outerRadius : innerRadius;
        star.push_back({
            canvasCenterX + r * cos(angle),
            canvasCenterY + r * sin(angle)
        });
    }
    
    GPaint starPaint(GColor::RGBA(0.9f, 0.9f, 1.0f, 0.9f));
    starPaint.setShader(shader);
    canvas->drawConvexPolygon(star.data(), star.size(), starPaint);

    // Surrounding hexagon
    const float hexRadius = 160.0f;
    GPoint hexagon[6];
    for (int i = 0; i < 6; i++) {
        float angle = (i * M_PI) / 3;
        hexagon[i] = {
            canvasCenterX + hexRadius * cos(angle),
            canvasCenterY + hexRadius * sin(angle)
        };
    }
    
    GPaint hexPaint(GColor::RGBA(0.4f, 0.6f, 0.8f, 0.5f));
    hexPaint.setShader(shader);
    canvas->drawConvexPolygon(hexagon, 6, hexPaint);

    // Small decorative squares at the corners
    const float squareSize = 30.0f;
    const float offset = 40.0f;
    const GPoint squares[][4] = {
        // Top left
        {{offset, offset}, 
         {offset + squareSize, offset},
         {offset + squareSize, offset + squareSize},
         {offset, offset + squareSize}},
        // Top right
        {{dim.width - offset - squareSize, offset},
         {dim.width - offset, offset},
         {dim.width - offset, offset + squareSize},
         {dim.width - offset - squareSize, offset + squareSize}},
        // Bottom right
        {{dim.width - offset - squareSize, dim.height - offset - squareSize},
         {dim.width - offset, dim.height - offset - squareSize},
         {dim.width - offset, dim.height - offset},
         {dim.width - offset - squareSize, dim.height - offset}},
        // Bottom left
        {{offset, dim.height - offset - squareSize},
         {offset + squareSize, dim.height - offset - squareSize},
         {offset + squareSize, dim.height - offset},
         {offset, dim.height - offset}}
    };

    GPaint squarePaint(GColor::RGBA(0.9f, 0.3f, 0.3f, 0.7f));

    return "Drawing using matrices and shaders";
}
void MyCanvas::drawMesh(const GPoint verts[], const GColor colors[], const GPoint texs[],
                       int count, const int indices[], const GPaint& paint) {
    // Early exit checks
    if (!paint.peekShader()) {
        texs = nullptr;
    }
    if (!colors && !texs) {
        return;
    }

    // Process each triangle
    for (int i = 0; i < count; ++i) {
        // Get indices for current triangle
        int p0 = indices[3 * i];
        int p1 = indices[3 * i + 1];
        int p2 = indices[3 * i + 2];

        // Transform vertices using canvas matrix
        GPoint transformedVerts[3];
        GPoint originalVerts[3] = {verts[p0], verts[p1], verts[p2]};
        fMatrix.mapPoints(transformedVerts, originalVerts, 3);

        // Set up colors if present
        GColor triColors[3];
        if (colors) {
            triColors[0] = colors[p0];
            triColors[1] = colors[p1];
            triColors[2] = colors[p2];
        }

        // Set up texture coordinates if present
        GPoint triTexs[3];
        if (texs) {
            triTexs[0] = texs[p0];
            triTexs[1] = texs[p1];
            triTexs[2] = texs[p2];
        }

        // Use the professor's drawTriangleInline function
        drawTriangleInline(transformedVerts, 
                          colors ? triColors : nullptr,
                          texs ? triTexs : nullptr,
                          paint, fBitmap);
    }
}

void MyCanvas::drawQuad(const GPoint verts[4], const GColor colors[4], const GPoint texs[4],
                        int level, const GPaint& paint) {
    
    // Early exit checks
    if (level < 1 || !verts) {
        return;
    }
    if (!colors && !texs) {
        return;
    }

    try {
        
        std::vector<GPoint> generatedVerts;
        std::vector<GColor> generatedColors;
        std::vector<GPoint> generatedTexs;
        std::vector<int> indices;

        int numVerts = (level + 1) * (level + 1);

        // Generate vertices and interpolate colors/texture coordinates
        for (int i = 0; i <= level; ++i) {
            float v = static_cast<float>(i) / level;
            for (int j = 0; j <= level; ++j) {
                float u = static_cast<float>(j) / level;

                float x = verts[0].x * (1 - u) * (1 - v) +
                          verts[1].x * u * (1 - v) +
                          verts[3].x * (1 - u) * v +
                          verts[2].x * u * v;
                float y = verts[0].y * (1 - u) * (1 - v) +
                          verts[1].y * u * (1 - v) +
                          verts[3].y * (1 - u) * v +
                          verts[2].y * u * v;
                generatedVerts.push_back({x, y});

                if (colors) {
                    // Safer color interpolation
                    float c00 = (1 - u) * (1 - v);
                    float c10 = u * (1 - v);
                    float c01 = (1 - u) * v;
                    float c11 = u * v;
                    
                    GColor interpolatedColor;
                    interpolatedColor.a = c00 * colors[0].a + c10 * colors[1].a + c01 * colors[3].a + c11 * colors[2].a;
                    interpolatedColor.r = c00 * colors[0].r + c10 * colors[1].r + c01 * colors[3].r + c11 * colors[2].r;
                    interpolatedColor.g = c00 * colors[0].g + c10 * colors[1].g + c01 * colors[3].g + c11 * colors[2].g;
                    interpolatedColor.b = c00 * colors[0].b + c10 * colors[1].b + c01 * colors[3].b + c11 * colors[2].b;
                    generatedColors.push_back(interpolatedColor);
                }
                if (texs) {
                    float tx = (1 - u) * (1 - v) * texs[0].x +
                               u * (1 - v) * texs[1].x +
                               (1 - u) * v * texs[3].x +
                               u * v * texs[2].x;
                    float ty = (1 - u) * (1 - v) * texs[0].y +
                               u * (1 - v) * texs[1].y +
                               (1 - u) * v * texs[3].y +
                               u * v * texs[2].y;
                    generatedTexs.push_back({tx, ty});
                }
            }
        }

        // Generate indices
        for (int i = 0; i < level; ++i) {
            for (int j = 0; j < level; ++j) {
                int idx = i * (level + 1) + j;

                // Upper triangle
                indices.push_back(idx);
                indices.push_back(idx + 1);
                indices.push_back(idx + level + 1);

                // Lower triangle
                indices.push_back(idx + 1);
                indices.push_back(idx + level + 2);
                indices.push_back(idx + level + 1);
            }
        }

       
        drawMesh(generatedVerts.data(),
                colors ? generatedColors.data() : nullptr,
                texs ? generatedTexs.data() : nullptr,
                indices.size() / 3, indices.data(), paint);
        

    } catch (const std::exception& e) {
        std::cout << "ERROR in drawQuad: " << e.what() << std::endl;
    } catch (...) {
        std::cout << "UNKNOWN ERROR in drawQuad" << std::endl;
    }
}


void computeBarycentric(float x, float y, const GPoint& v0, const GPoint& v1, const GPoint& v2,
                        float& alpha, float& beta, float& gamma) {
    float denom = (v1.y - v2.y) * (v0.x - v2.x) + (v2.x - v1.x) * (v0.y - v2.y);
    alpha = ((v1.y - v2.y) * (x - v2.x) + (v2.x - v1.x) * (y - v2.y)) / denom;
    beta  = ((v2.y - v0.y) * (x - v2.x) + (v0.x - v2.x) * (y - v2.y)) / denom;
    gamma = 1.0f - alpha - beta;
}

GPixel colorToPixel(const GColor& color) {
    int a = static_cast<int>(color.a * 255 + 0.5f);
    int r = static_cast<int>(color.r * color.a * 255 + 0.5f);
    int g = static_cast<int>(color.g * color.a * 255 + 0.5f);
    int b = static_cast<int>(color.b * color.a * 255 + 0.5f);
    return GPixel_PackARGB(a, r, g, b);
}

GPixel blendPixel(const GPixel& dst, const GPixel& src, GBlendMode mode) {
    if (mode == GBlendMode::kSrc) {
        return src; // Simply replace the destination with the source
    }

    if (mode == GBlendMode::kSrcOver) {
        int srcA = GPixel_GetA(src);
        int dstA = GPixel_GetA(dst);

        int outA = srcA + GDiv255((255 - srcA) * dstA);
        int outR = GPixel_GetR(src) + GDiv255((255 - srcA) * GPixel_GetR(dst));
        int outG = GPixel_GetG(src) + GDiv255((255 - srcA) * GPixel_GetG(dst));
        int outB = GPixel_GetB(src) + GDiv255((255 - srcA) * GPixel_GetB(dst));

        return GPixel_PackARGB(outA, outR, outG, outB);
    }

    // Add other blend modes as needed, or assert invalid mode
    return src; // Fallback to source if no valid mode is provided
}

void drawTriangleInline(const GPoint verts[3], const GColor colors[3], const GPoint texs[3], 
                        const GPaint& paint, const GBitmap& device) {
    // Compute bounds with proper rounding
    int left = std::max(0, static_cast<int>(std::floor(std::min({verts[0].x, verts[1].x, verts[2].x}))));
    int right = std::min(device.width(), static_cast<int>(std::ceil(std::max({verts[0].x, verts[1].x, verts[2].x}))));
    int top = std::max(0, static_cast<int>(std::floor(std::min({verts[0].y, verts[1].y, verts[2].y}))));
    int bottom = std::min(device.height(), static_cast<int>(std::ceil(std::max({verts[0].y, verts[1].y, verts[2].y}))));

    // Precompute edge equations
    float dx12 = verts[1].x - verts[0].x;
    float dy12 = verts[1].y - verts[0].y;
    float dx23 = verts[2].x - verts[1].x;
    float dy23 = verts[2].y - verts[1].y;
    float dx31 = verts[0].x - verts[2].x;
    float dy31 = verts[0].y - verts[2].y;

    // Get shader
    GShader* shader = paint.peekShader();
    bool useShader = shader && shader->setContext(GMatrix());

    // Sample at pixel centers
    float px = 0.5f;
    float py = 0.5f;

    for (int y = top; y < bottom; ++y) {
        GPixel* row = device.getAddr(0, y);
        float fy = y + py;

        for (int x = left; x < right; ++x) {
            float fx = x + px;

            // Compute barycentric coordinates
            float w0 = (dx23 * (fy - verts[1].y) - dy23 * (fx - verts[1].x));
            float w1 = (dx31 * (fy - verts[2].y) - dy31 * (fx - verts[2].x));
            float w2 = (dx12 * (fy - verts[0].y) - dy12 * (fx - verts[0].x));

            // Area of triangle
            float area = dx23 * dy31 - dx31 * dy23;
            if (area == 0) continue;

            // Normalize barycentric coordinates
            float invArea = 1.0f / area;
            w0 *= invArea;
            w1 *= invArea;
            w2 *= invArea;

            // Check if point is inside triangle
            if (w0 >= 0 && w1 >= 0 && w2 >= 0) {
                GPixel srcPixel;

                if (useShader && texs) {
                    // Compute texture coordinates
                    float tx = w0 * texs[0].x + w1 * texs[1].x + w2 * texs[2].x;
                    float ty = w0 * texs[0].y + w1 * texs[1].y + w2 * texs[2].y;
                    shader->shadeRow(tx, ty, 1, &srcPixel);
                } else if (colors) {
                    // Interpolate colors
                    float a = w0 * colors[0].a + w1 * colors[1].a + w2 * colors[2].a;
                    float r = w0 * colors[0].r + w1 * colors[1].r + w2 * colors[2].r;
                    float g = w0 * colors[0].g + w1 * colors[1].g + w2 * colors[2].g;
                    float b = w0 * colors[0].b + w1 * colors[1].b + w2 * colors[2].b;

                    // Clamp values and premultiply
                    a = std::max(0.0f, std::min(1.0f, a));
                    r = std::max(0.0f, std::min(1.0f, r)) * a;
                    g = std::max(0.0f, std::min(1.0f, g)) * a;
                    b = std::max(0.0f, std::min(1.0f, b)) * a;

                    srcPixel = GPixel_PackARGB(
                        static_cast<uint8_t>(a * 255 + 0.5f),
                        static_cast<uint8_t>(r * 255 + 0.5f),
                        static_cast<uint8_t>(g * 255 + 0.5f),
                        static_cast<uint8_t>(b * 255 + 0.5f)
                    );
                }

                row[x] = blendPixel(row[x], srcPixel, paint.getBlendMode());
            }
        }
    }
}


// std::shared_ptr<GPath> final_addStrokedLine(GPath* path, GPoint p0, GPoint p1, float width, bool roundCap) {
//     float slope = (p1.y - p0.y) / (p1.x - p0.x);
//     float perp = -1 / slope;

//     float corner_angle = atan(perp);

//     float corner_dx = cos(corner_angle) * width / 2;
//     float corner_dy = sin(corner_angle) * width / 2;

//     GPoint c0 = {p0.x + corner_dx, p0.y + corner_dy};
//     GPoint c1 = {p0.x - corner_dx, p0.y - corner_dy};
//     GPoint c2 = {p1.x - corner_dx, p1.y - corner_dy};
//     GPoint c3 = {p1.x + corner_dx, p1.y + corner_dy};

//     GPathBuilder builder;
//     builder.moveTo(c0);
//     builder.lineTo(c1);
//     builder.lineTo(c2);
//     builder.lineTo(c3);

//     return builder.detach();
// }


