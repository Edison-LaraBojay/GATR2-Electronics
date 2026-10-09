/* apriltag_c.h
 * Plain C boundary around the vendored AprilRobotics detector. C++ code
 * never includes apriltag headers directly: on Windows their pthread shim
 * collides with the standard library's own pthread types inside one
 * translation unit, and keeping one boundary on every platform keeps the
 * adapter honest about what it uses.
 *
 * Detection callbacks receive corners in the captured image with the
 * upstream order: p[0] bottom-left of the printed tag, then
 * counter-clockwise as displayed. Poses are solved from caller-supplied
 * (already undistorted) corners against the width_at_border square of the
 * given physical size, in the detector-native optical frame (+x right,
 * +y down, +z into the tag).
 */

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct nav_apriltag_detector nav_apriltag_detector;

typedef struct {
    const char* family;
    int         id;
    int         hamming;
    float       decision_margin;
    double      corners[4][2];
    double      center[2];
} nav_apriltag_detection;

typedef void (*nav_apriltag_sink)(void* user, const nav_apriltag_detection* detection);

typedef struct {
    double corners[4][2]; /* undistorted pixels, upstream order */
    double tagsize_m;     /* width_at_border square */
    double fx, fy, cx, cy;
    int    iterations;
} nav_apriltag_pose_request;

typedef struct {
    double R[9]; /* row major, T_optical_tag_native */
    double t[3];
    double err1;
    double err2;       /* object-space error of the alternate minimum, or -1 */
    int    has_second; /* a competing minimum existed */
} nav_apriltag_pose_result;

/* Detector with the upstream tunables; families are added separately. */
nav_apriltag_detector* nav_apriltag_create(double quad_decimate, double quad_sigma, int nthreads,
                                           int refine_edges, double decode_sharpening);
void                   nav_apriltag_destroy(nav_apriltag_detector* detector);

/* 0 on success, -1 unknown family, -2 already added. */
int nav_apriltag_add_family(nav_apriltag_detector* detector, const char* family);

/* Runs the detector over a packed or strided Y8 buffer. The buffer may be
 * written in place by the upstream detector; pass a private copy. Returns
 * the number of detections, or -1 on failure. */
int nav_apriltag_detect(nav_apriltag_detector* detector, int width, int height, int stride,
                        uint8_t* buffer, nav_apriltag_sink sink, void* user);

/* Pose from four undistorted corners. Returns 0 on success, -1 when the
 * homography or solve failed. */
int nav_apriltag_pose(const nav_apriltag_pose_request* request, nav_apriltag_pose_result* out);

/* Family geometry: the corner square spans width_at_border of total_width
 * printed cells. 0 on success, -1 unknown family. */
int nav_apriltag_family_geometry(const char* family, int* width_at_border, int* total_width);

/* The printed tag as one byte per cell (0 black, 255 white), total_width
 * cells per side, written row major into cells (capacity bytes). Returns
 * total_width, 0 when capacity is too small, -1 unknown family, -2 bad id. */
int nav_apriltag_render(const char* family, int id, uint8_t* cells, int capacity);

/* Supported family names, NULL past the end. */
const char* nav_apriltag_supported_family(int index);

#ifdef __cplusplus
}
#endif
