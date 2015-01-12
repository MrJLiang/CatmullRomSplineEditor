/*
    CSC 486 Summer 2014
    Ryan Guy
*/

#include "catmullromspline.h"
#include <QtDebug>

CatmullRomSpline::CatmullRomSpline()
{
}

CatmullRomSpline::CatmullRomSpline(QVector<glm::vec4> &control_points):
    controlPoints(control_points)
{
    initializeGeometryVectors();
    initializeBasisMatrix();
    initializeSegmentLengths();
    initializeArcLengthTable();
}

CatmullRomSpline::~CatmullRomSpline()
{
    //delete &controlPoints;
}

void CatmullRomSpline::initializeGeometryVectors() {

    // compute tangents
    // A tangent of a point is computed as a factor of the vector
    // starting at the point previous point and ending at the point's next point
    glm::vec3 tangents[MAX_NUM_CONTROL_POINTS - 2];
    for (int i=1; i < controlPoints.size() - 1; i++) {
        glm::vec4 p1 = controlPoints[i-1];
        glm::vec4 p2 = controlPoints[i+1];

        float dx = 0.5f * (p2.x - p1.x);
        float dy = 0.5f * (p2.y - p1.y);
        float dz = 0.5f * (p2.z - p1.z);
        tangents[i-1] = glm::vec3(dx, dy, dz);
    }

    // construct geomentry vectors
    // Example: A geometry vector for the x component is in form:
    //         {p1x, p2x, t1x, t2x}
    //         where p1 and p2 are the start point and end point of a segment
    //         and t1 and t2 are their tangent vectors
    for (int i=0; i < controlPoints.size() - 3; i++) {
        glm::vec4 p1 = controlPoints[i + 1];
        glm::vec4 p2 = controlPoints[i + 2];
        glm::vec3 t1 = tangents[i];
        glm::vec3 t2 = tangents[i + 1];
        geometryVectorsX.append(glm::vec4(p1.x, p2.x, t1.x, t2.x));
        geometryVectorsY.append(glm::vec4(p1.y, p2.y, t1.y, t2.y));
        geometryVectorsZ.append(glm::vec4(p1.z, p2.z, t1.z, t2.z));
    }
}

void CatmullRomSpline::initializeBasisMatrix() {
    // System of equations for ax^3 + bx^2 + cx + d
    // Example for x:
    // x(0) =  p0x = [0 0 0 1]*Cx  // x component of p0
    // x(1) =  p3x = [1 1 1 1]*Cx  // x component of p3
    // x'(0) = t0x = [0 0 1 0]*Cx  // tangent x component to p0
    // x'(1) = t3x = [3 2 1 1]*Cx  // tangent x component to p3

    glm::mat4 invBasisMatrix = glm::mat4(glm::vec4(0.0f, 0.0f, 0.0f, 1.0f),
                                         glm::vec4(1.0f, 1.0f, 1.0f, 1.0f),
                                         glm::vec4(0.0f, 0.0f, 1.0f, 0.0f),
                                         glm::vec4(3.0f, 2.0f, 1.0f, 0.0f));

    // invert and transpose to get basis matrix
    basisMatrix = glm::transpose(glm::inverse(invBasisMatrix));
}

glm::vec3 CatmullRomSpline::evaluatePointOnSegment(float t, int segmentIdx) {


    glm::vec4 paramVect = glm::vec4(t*t*t, t*t, t, 1.0f);
    glm::vec4 geomX = geometryVectorsX[segmentIdx];
    glm::vec4 geomY = geometryVectorsY[segmentIdx];
    glm::vec4 geomZ = geometryVectorsZ[segmentIdx];

    // point = paramVect * basisMatrix * geometryVector
    float x = glm::dot(paramVect, basisMatrix * geomX);
    float y = glm::dot(paramVect, basisMatrix * geomY);
    float z = glm::dot(paramVect, basisMatrix * geomZ);

    return glm::vec3(x, y, z);
}

void CatmullRomSpline::initializeSegmentLengths() {
    float totalLength = 0.0f;
    float step = 1.0f / (POINTS_PER_SEGMENT - 1);
    for (int segIdx=0; segIdx < controlPoints.size() - 3; segIdx++) {
        // calculate length of a segment
        float t = step;
        float segLen = 0.0f;
        glm::vec3 lastPoint = evaluatePointOnSegment(0.0f, segIdx);
        for (int i=1; i < POINTS_PER_SEGMENT; i++) {
            glm::vec3 p = evaluatePointOnSegment(t, segIdx);
            float stepDist = glm::distance(p, lastPoint);
            segLen += stepDist;

            lastPoint = p;
            t += step;
        }

        totalLength += segLen;
        segmentLengths.append(segLen);
    }
    this->totalLength = totalLength;
}

void CatmullRomSpline::initializeArcLengthTable() {
    float step = 1.0f / (POINTS_PER_SEGMENT - 1);
    float currentLength = 0.0f;

    for (int segIdx=0; segIdx < controlPoints.size() - 3; segIdx++) {
        float t = step;
        float segLen = 0.0f;
        glm::vec3 lastPoint = evaluatePointOnSegment(0.0f, segIdx);
        arcLengthTable.append(glm::vec3(currentLength, segIdx, 0.0f));
        for (int i=1; i < POINTS_PER_SEGMENT; i++) {

            glm::vec3 p = evaluatePointOnSegment(t, segIdx);
            float stepDist = glm::distance(p, lastPoint);
            segLen += stepDist;
            currentLength += stepDist;

            arcLengthTable.append(glm::vec3(currentLength, segIdx, t));

            lastPoint = p;
            t += step;
        }
    }
}

int CatmullRomSpline::getSegmentIndexAtArcLength(float len) {
    float currentLength = 0.0f;
    int i = 0;
    for (i=0; i < segmentLengths.size(); i++) {
        currentLength += segmentLengths[i];
        if (len <= currentLength) {
            return i;
        }
    }
    return i-1;
}

glm::vec3 CatmullRomSpline::getPointAtArcLength(float len) {
  int segIdx = getSegmentIndexAtArcLength(len);
  int min = segIdx * POINTS_PER_SEGMENT;
  int max = min + POINTS_PER_SEGMENT - 1;
  for (int i=min; i < max; i++) {
    glm::vec3 data2 = arcLengthTable[i+1];
    if (len <= data2.x) {
        glm::vec3 data1 = arcLengthTable[i];

        float minLen = data1.x;
        float maxLen = data2.x;
        float minT = data1.z;
        float maxT = data2.z;
        float ratio = (len - minLen) / (maxLen - minLen);
        float t = minT + ratio * (maxT - minT);
        return evaluatePointOnSegment(t, segIdx);
    }
  }

  // last control point
  glm::vec4 p = controlPoints[controlPoints.size()-2];
  return glm::vec3(p.x, p.y, p.z);
}

glm::vec3 CatmullRomSpline::interpolateForT(float t) {
    if (t < 0.0f) {
        t = 0.0f;
    } else if ( t > 1.0f) {
        t = 1.0f;
    }
    return getPointAtArcLength(t * totalLength);
}















