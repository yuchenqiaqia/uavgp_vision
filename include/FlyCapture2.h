#ifndef PGR_FC2_FLYCAPTURE2_H
#define PGR_FC2_FLYCAPTURE2_H

//=============================================================================
// Global header file for FlyCapture2.
//
// By including this file, all required header files for full FlyCapture2
// operation will be included automatically. It is recommended that this file
// be used instead of manually including individual header files.
//=============================================================================

//=============================================================================
// Platform-specific definitions
//=============================================================================
#include "FlyCapture2Platform.h"

//=============================================================================
// Global definitions
//=============================================================================
#include "FlyCapture2Defs.h"

//=============================================================================
// PGR Error class
//=============================================================================
#include "Error.h"

//=============================================================================
// FlyCapture2 classes
//=============================================================================
#include "BusManager.h"
#include "Camera.h"
#include "GigECamera.h"
#include "Image.h"

//=============================================================================
// Utility classes
//=============================================================================
#include "Utilities.h"
#include "AVIRecorder.h"
#include "TopologyNode.h"
#include "ImageStatistics.h"

#endif // PGR_FC2_FLYCAPTURE2_H

