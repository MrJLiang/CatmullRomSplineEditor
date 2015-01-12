/*
    CSC 486 Summer 2014
    Ryan Guy
*/

/****************************************************************************
**
** Copyright (C) 2013 Digia Plc and/or its subsidiary(-ies).
** Contact: http://www.qt-project.org/legal
**
** This file is part of the examples of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:BSD$
** You may use this file under the terms of the BSD license as follows:
**
** "Redistribution and use in source and binary forms, with or without
** modification, are permitted provided that the following conditions are
** met:
**   * Redistributions of source code must retain the above copyright
**     notice, this list of conditions and the following disclaimer.
**   * Redistributions in binary form must reproduce the above copyright
**     notice, this list of conditions and the following disclaimer in
**     the documentation and/or other materials provided with the
**     distribution.
**   * Neither the name of Digia Plc and its Subsidiary(-ies) nor the names
**     of its contributors may be used to endorse or promote products derived
**     from this software without specific prior written permission.
**
**
** THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
** "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
** LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
** A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
** OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
** SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
** LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
** DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
** THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
** (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
** OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE."
**
** $QT_END_LICENSE$
**
****************************************************************************/

#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QMatrix4x4>
#include "glm/glm.hpp"
#include "catmullromspline.h"

#define EPSILON 0.0000001              /**< For comparing floats */
#define LINES_PER_SEGMENT  50          /**< Samples points per segment of spline */
#define DEFAULT_ANIMATION_TIME  2.3    /**< in seconds */
#define DEFAULT_ACCELERATION_TIME 0.3  /**< Time animation spends accelerating/deceleratting */
#define DELTA_TIME   0.05              /**< time since last frame */

class QtLogo;

/**
  * @class  GLWidget
  * @brief  our OpenGL view derived from QGLWidget.
  * We have to override several functions for our
  * application-specific OpenGL drawing functionality
  */
class GLWidget : public QGLWidget
{
    Q_OBJECT

public:
    GLWidget(QWidget *parent = 0);
    ~GLWidget();

    QSize minimumSizeHint() const;
    QSize sizeHint() const;

protected:
    void initializeGL();
    void paintGL();                             /**< Called whenever widget is being redrawn */
    void resizeGL(int width, int height);       /**< Called when window is resized */

    // Mouse input callbacks
    void mousePressEvent(QMouseEvent *event);   /**< Left click to add point, Right click to remove point */
    void mouseReleaseEvent(QMouseEvent *event); /**< Point is actually added when the mouse is released */
    void mouseMoveEvent(QMouseEvent *event);    /**< Left click and drag to move point*/

private:
    QMatrix4x4  viewMatrix;             /**< For setting camera lookat */
    QVector<glm::vec4>  controlPoints;  /**< For managing control points*/
    CatmullRomSpline catmullRomSpline;  /**< The spline object */

    float pointRadius;              /**< Points have a radius so that mouse can select points */
    int selectedPointIndex;         /**< Index of selected control point for QVector controlPoints */
    bool isPointSelected;           /**< True if a point is slected by the mouse, false otherwise */
    bool isPointMoving;             /**< True if a point is selected and it has been moved */
    float animationTimePerSegment;  /**< Length of animation to add per section of spline (in seconds) */
    float currentAnimationTime;     /**< Used to update animation */
    float animationProgress;        /**< Parameterized progress of animation in range [0,1] */
    float accelerationTime;         /**< Time that animation spends accelerating/decelerating */

    /**
     *	Function is for determining whether the addition of a new point (x, y, z) will cause
     *  some 3 or more controlPoints to become collinear
     *	@param	x x coordinate of point to add
     *	@param	y y coordinate of point to add
     *	@param	z z coordinate of point to add
     *	@return	returns whether the addition of point (x, y, z) causes 3 or more points to become collinear
     */
    bool isPointCollinearWithControlPoints(float x, float y, float z);

    /**
     *	Adds a control point to controlPoints if possible
     *	@param	x x coordinate of point to add
     *	@param	y y coordinate of point to add
     *	@param	z z coordinate of point to add
     */
    void addControlPoint(float x, float y, float z);

    /**
     *	Removes a control point near position (x, y) if that point exists
     *	@param	x x coordinate of point removal area
     *	@param	y y coordinate of point removal area
     */
    void removeControlPoint(float x, float y);

    /**
     *	Moves a control point to position (destX, destY, destZ) if possible
     *  @param	pIndex Index of point in controlPoints that is to be moved
     *	@param	destX x coordinate of destination
     *	@param	destY y coordinate of destination
     *	@param	destZ z coordinate of destination
     */
    void moveControlPoint(int pIndex, float destX, float destY, float destZ);

    /**
     *  @return	returns whether floats f1 and f2 can be considered equal
     */
    bool floatIsEqual(float f1, float f2);

    /**
     *  @param	x x coordinate of area to look for point
     *  @param	y y coordinate of area to look for point
     *  @return	returns index of controlPoint point in the area of (x, y)
     *          or returns -1 if there is no such point
     */
    int getSelectedPoint(float x, float y);

    /**< called to update catmullRomSpline with a changed set of controlPoints */
    void updateCatmullRomSpline();

    /**< called to restart animation to beginning of path */
    void resetAnimation();

    /**< called each redraw to update position of animation */
    void updateAnimation();

    /**< called to update camera view for each redraw */
    void updateViewport();

    void drawCatmullRomSpline();    /**< draws path of spline */
    void drawControlPoints();       /**< draws each control point */
    void drawSelectedPoint();       /**< draws point selection */

    /**
     *  For ease in/out motion. Formula from interpolation lecture slides
     *  @param	t  time to calculate position for in range [0,1]
     *  @param	t1 time at which motion stops accelerating in range [0,1] 0 < t1 < t2
     *  @param	t2 time at which motion starts decelerating in range [0,1] t1 < t2 < 1
     *  @return	position of point at time t
     */
    float ease(float t, float t1, float t2);


};

#endif
