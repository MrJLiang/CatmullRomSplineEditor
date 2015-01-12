/*
    CSC 486 Summer 2014
    Ryan Guy
*/

#ifndef CATMULLROMSPLINE_H
#define CATMULLROMSPLINE_H

#include <QVector>
#include "glm/glm.hpp"

#define MAX_NUM_CONTROL_POINTS   100
#define POINTS_PER_SEGMENT       25   // number of points to sample per segment


/**
  * @class  CatmullRomSpline
  * @brief  Generates a Catmull-Rom spline in 3d space based on a set of control points.
  *         Values on spline can be queried by values in range [0,1] parameterized by arclength
  */
class CatmullRomSpline
{
public:
    CatmullRomSpline();
    CatmullRomSpline(QVector<glm::vec4> & control_points);
    ~CatmullRomSpline();

public:

    /**
     *	Function returns point on spline at value t
     *  parameterized by arclength
     *  @param t parameterized value over entire spline in range [0,1]
     *	@return	returns (x,y,z) at value t
     */
    glm::vec3 interpolateForT(float t);

private:
    QVector<glm::vec4>  controlPoints;     /**< Stores control poitns of spline */

    /**< Stores geometry vectors for each (x,y,z) component */
    QVector<glm::vec4>  geometryVectorsX;
    QVector<glm::vec4>  geometryVectorsY;
    QVector<glm::vec4>  geometryVectorsZ;

    QVector<float>  segmentLengths;        /**< Stores length of each segment */
    QVector<glm::vec3> arcLengthTable;     /**< stores table of arc lengths, spline segment index,
                                                and spline parameterized value in a vector in order of arclength */
    float totalLength; /**< Total length of path */

    glm::mat4 basisMatrix;  /**< Stores basis matrix */

    /**< Functions for initializing the spline */
    void initializeGeometryVectors();
    void initializeBasisMatrix();
    void initializeSegmentLengths();
    void initializeArcLengthTable();

    /**
     *	Function is for evaluating a point on a spline segment
     *	@param	t parameterized value along segment in range [0,1]
     *	@param	segmentIdx index of segment to evaluate point on
     *	@return	returns (x,y,z) vector of point
     */
    glm::vec3 evaluatePointOnSegment(float t, int segmentIdx);

    /**
     *	Function returns segment index of point at distance len
     *	@param	len length since start of path that point end on
     *	@return	returns segment index
     */
    int getSegmentIndexAtArcLength(float len);

    /**
     *	Function returns point on path that is at length len
     *	@param	len length since start the point ends on
     *	@return	returns (x,y,z) point at length len
     */
    glm::vec3 getPointAtArcLength(float len);

};

#endif // CATMULLROMSPLINE_H
