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

#include <QtWidgets>
#include <QtOpenGL>

#include <math.h>

#include "glwidget.h"
#include "catmullromspline.h"
#include <QMatrix4x4>

#include "glm/glm.hpp"

#ifndef GL_MULTISAMPLE
#define GL_MULTISAMPLE  0x809D
#endif

//! [0]
GLWidget::GLWidget(QWidget *parent)
    : QGLWidget(QGLFormat(QGL::SampleBuffers), parent),
      viewMatrix(),
      controlPoints(),
      catmullRomSpline()
{

    pointRadius = 10.0f;
    isPointSelected = false;
    isPointMoving = false;

    animationTimePerSegment = (float)DEFAULT_ANIMATION_TIME;
    currentAnimationTime = 0.0f;
    accelerationTime = (float)DEFAULT_ACCELERATION_TIME;
}

GLWidget::~GLWidget()
{
}

QSize GLWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}

QSize GLWidget::sizeHint() const
{
    return QSize(800, 800);
}

void GLWidget::initializeGL()
{
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
}

void GLWidget::drawCatmullRomSpline(){
    if (controlPoints.size() < 4) {
        return;
    }

    // Draw points
    float invWidth = 1.0f / (float)this->width();
    float invHeight = 1.0f / (float)this->height();
    int n = (controlPoints.size() - 3) * LINES_PER_SEGMENT;
    glColor3f(0.0f, 0.0f, 1.0f);
    glLineWidth(2.0f);
    glEnable(GL_LINE_SMOOTH);
    glBegin(GL_LINES);
        glm::vec3 p1 = catmullRomSpline.interpolateForT(0.0f);
        for (int i=1; i <= n; i++) {
            float t = (float)i / (float)n;
            glm::vec3 p2 = catmullRomSpline.interpolateForT(t);
            glVertex3d(p1.x * invWidth, p1.y * invHeight, p1.z);
            glVertex3d(p2.x * invWidth, p2.y * invHeight, p2.z);
            p1 = p2;
        }

    glEnd();

    // draw lines between beginning and end control points
    glColor3f(0.8f, 0.8f, 0.8f);
    glBegin(GL_LINES);
        glm::vec4 v1 = controlPoints[0];
        glm::vec4 v2 = controlPoints[1];
        glm::vec4 v3 = controlPoints[controlPoints.size() - 2];
        glm::vec4 v4 = controlPoints[controlPoints.size() - 1];
        glVertex3d(v1.x * invWidth, v1.y * invHeight, v1.z);
        glVertex3d(v2.x * invWidth, v2.y * invHeight, v2.z);
        glVertex3d(v3.x * invWidth, v3.y * invHeight, v3.z);
        glVertex3d(v4.x * invWidth, v4.y * invHeight, v4.z);
    glEnd();

    // Draw animation position
    glm::vec3 animPos = catmullRomSpline.interpolateForT(animationProgress);
    glColor3f(1.0f, 0.0f, 0.0f);
    glPointSize(20.0f);
    glEnable(GL_NICEST);
    glBegin(GL_POINTS);
        glVertex3d(animPos.x * invWidth, animPos.y * invHeight, animPos.z);
    glEnd();
}

void GLWidget::drawControlPoints() {
    float invWidth = 1.0f / (float)this->width();
    float invHeight = 1.0f / (float)this->height();
    glColor3f(1.0f, 0.0f, 0.0f);
    glPointSize(6.0f);
    glEnable(GL_NICEST);
    glBegin(GL_POINTS);
        for (int i=0; i < controlPoints.size(); i++) {
            glm::vec4 p = controlPoints[i];
            glVertex3d(p.x * invWidth, p.y * invHeight, p.z);
        }
    glEnd();
}

void GLWidget::drawSelectedPoint() {
    if (!isPointMoving) {
        return;
    }

    float invWidth = 1.0f / (float)this->width();
    float invHeight = 1.0f / (float)this->height();
    float x = controlPoints[selectedPointIndex].x * invWidth;
    float y = controlPoints[selectedPointIndex].y * invHeight;
    glColor3f(0.0f, 0.2f, 1.0f);
    glLineWidth(2.0f);
    glEnable(GL_LINE_SMOOTH);
    glBegin(GL_LINE_LOOP);
        float radius = 10.0f;
        int numSegments = 50;
        float radIncrement = 6.283185 / (float) numSegments;
        for (float i=0.0f; i < 6.283185; i = i + radIncrement)
        {
           glVertex2f(x + cos(i)*radius*invWidth, y + sin(i)*radius*invHeight);
        }
    glEnd();

}

void GLWidget::updateViewport() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    viewMatrix.setToIdentity();

    float offx = 0.5f;
    float offy = 0.5f;
    viewMatrix.lookAt(QVector3D(offx, offy, -4.0f),
                      QVector3D(offx, offy, 0.0f),
                      QVector3D(0.0f, -1.0f, 0.0f));
    glLoadMatrixf(viewMatrix.data());
}

void GLWidget::paintGL()
{
    updateAnimation();
    updateViewport();
    drawCatmullRomSpline();
    drawControlPoints();
    drawSelectedPoint();
}

void GLWidget::updateCatmullRomSpline() {
    if (controlPoints.size() >= 4) {
        this->catmullRomSpline = CatmullRomSpline(controlPoints);
        resetAnimation();
    }
}

float GLWidget::ease(float t, float t1, float t2) {
    float v0;
    float d;

    v0 = 2/(1+t2-t1); /* constant velocity attained */
    if (t<t1) {
        d = v0*t*t/(2*t1);
    } else {
        d = v0*t1/2;
        if (t<t2) {
            d += (t-t1)*v0;
        } else {
            d += (t2-t1)*v0;
            d += (t-t*t/2-t2+t2*t2/2)*v0/(1-t2);
        }
    }
    return (d);
}

void GLWidget::resetAnimation() {
    currentAnimationTime = 0.0f;
}

void GLWidget::updateAnimation() {
    if (controlPoints.size() < 4) {
        return;
    }

    currentAnimationTime += DELTA_TIME;
    int numSegments = (float)(controlPoints.size() - 3);
    float animTime = animationTimePerSegment*numSegments;
    if (currentAnimationTime > animTime) {
        currentAnimationTime = 0.0f;
    }

    float t = currentAnimationTime / animTime;
    float t1 = 0.0f + accelerationTime;
    float t2 = 1.0f - accelerationTime;
    animationProgress = ease(t, t1, t2);
}


bool GLWidget::floatIsEqual(float f1, float f2) {
    if (fabs(f1 - f2) <= EPSILON) {
        return true;
    }
    return false;
}

void GLWidget::resizeGL(int width, int height)
{
    int side = qMin(width, height);
    glViewport((width - side) / 2, (height - side) / 2, side, side);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
#ifdef QT_OPENGL_ES_1
    glOrthof(-0.5, +0.5, -0.5, +0.5, 4.0, 15.0);
#else
    glOrtho(-0.5, +0.5, -0.5, +0.5, 4.0, 15.0);
#endif
    glMatrixMode(GL_MODELVIEW);
}

// check if point (x, y, z) is collinear with any other 2 points in controlPoints
bool GLWidget::isPointCollinearWithControlPoints(float x, float y, float z) {
    for (int j=0; j < controlPoints.size(); j++) {
        for (int i = 0; i < j; i++) {
            glm::vec3 v1 = glm::vec3(x - controlPoints[i].x,
                                     y - controlPoints[i].y,
                                     z - controlPoints[i].z);
            glm::vec3 v2 = glm::vec3(x - controlPoints[j].x,
                                     y - controlPoints[j].y,
                                     z - controlPoints[j].z);
            glm::vec3 cross = glm::cross(v1, v2);
            float crossLenSqr = cross.x * cross.x + cross.y * cross.y + cross.z * cross.z;

            if (floatIsEqual(crossLenSqr, 0.0f)) {
                return true;
            }
        }
    }
    return false;
}

void GLWidget::addControlPoint(float x, float y, float z) {
    // don't add this point if it is equal to another point in controlPoints
    for (int i = 0; i < controlPoints.size(); i++) {
        glm::vec4 p = controlPoints[i];
        if(floatIsEqual(x, p.x) && floatIsEqual(y, p.y) && floatIsEqual(z, p.z)) {
            return;
        }
    }

    // if point is collinear with 2 other points, shift position until it is not
    int maxShifts = 10;
    int shifts = 0;
    while (isPointCollinearWithControlPoints(x, y, z)) {
        if (shifts <= maxShifts / 2) {
          x++;
        } else {
          y++;
        }
        shifts++;
        if (shifts == maxShifts) {
            return;
        }
    }

    controlPoints.append(glm::vec4(x, y, z, 1.0f));
    updateCatmullRomSpline();
}

// removes control point near (x, y) if it exists
void GLWidget::removeControlPoint(float x, float y) {
    int selectIndex = getSelectedPoint(x, y);
    if (selectIndex != -1) {
        controlPoints.remove(selectIndex);
        updateCatmullRomSpline();
    }
}

// move point p to position (destX, destY)
void GLWidget::moveControlPoint(int pIndex, float destX, float destY, float destZ) {
    // dont move if destination is at an existing point
    for (int i = 0; i < controlPoints.size(); i++) {
        if (pIndex == i) {
            continue;
        }

        glm::vec4 v = controlPoints[i];
        if(floatIsEqual(destX, v.x) && floatIsEqual(destY, v.y) && floatIsEqual(destZ, v.z)) {
            return;
        }
    }

    // dont move if (destX, destY, destZ) is collinear with control points
    if (isPointCollinearWithControlPoints(destX, destY, destZ)) {
        return;
    }

    controlPoints[pIndex] = glm::vec4(destX, destY, destZ, 1.0f);
    updateCatmullRomSpline();
}

// returns index of point near (x, y), or -1 if there is no point
int GLWidget::getSelectedPoint(float x, float y) {
    float rsq = pointRadius * pointRadius;
    for (int i = 0; i < controlPoints.size(); i++) {
        glm::vec4 p = controlPoints[i];
        float dx = x - p.x;
        float dy = y - p.y;
        if (dx*dx + dy*dy <= rsq) {
            return i;
        }
    }
    return -1;
}

void GLWidget::mousePressEvent(QMouseEvent *event)
{
    // right click to remove points
    if (event->button() == Qt::RightButton) {
        QPoint mpos = event->pos();
        removeControlPoint((float)mpos.x(), (float)mpos.y());
    }

    // left click and drag to to move points
    if (event->button() == Qt::LeftButton) {
        QPoint mpos = event->pos();
        int selectIndex = getSelectedPoint((float)mpos.x(), (float)mpos.y());
        if (selectIndex != -1) {
            selectedPointIndex = selectIndex;
            isPointSelected = true;
        } else {
            isPointSelected = false;
        }
    }
}

void GLWidget::mouseReleaseEvent(QMouseEvent *event)
{
    // left click to add points
    if (!isPointMoving && event->button() == Qt::LeftButton) {
        QPoint mpos = event->pos();
        addControlPoint((float)mpos.x(), (float)mpos.y(), 0.0f);
    }

    isPointMoving = false;
    isPointSelected = false;
}

void GLWidget::mouseMoveEvent(QMouseEvent *event)
{

    if (isPointSelected && (event->buttons() & Qt::LeftButton)) {
        float x = (float)event->x();
        int y = (float)event->y();
        moveControlPoint(selectedPointIndex, x, y, 0.0f);
        isPointMoving = true;
    }
}
