#pragma once

#include <QImage>

enum class VisionOverlay { Body, Hands, Face };

// Operates on an RGB frame already obtained from the Kinect transport.
QImage annotateWithVision(const QImage& frame, VisionOverlay overlay);
