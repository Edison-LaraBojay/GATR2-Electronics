/* apriltag_c.c */

#include "impl/resources/apriltag_c.h"

#include <math.h>
#include <string.h>

#include "apriltag.h"
#include "apriltag_pose.h"
#include "common/homography.h"
#include "common/image_u8.h"
#include "common/matd.h"
#include "common/zarray.h"
#include "tag16h5.h"
#include "tag25h9.h"
#include "tag36h10.h"
#include "tag36h11.h"
#include "tagCircle21h7.h"
#include "tagStandard41h12.h"

typedef struct {
    const char* name;
    apriltag_family_t* (*create)(void);
    void (*destroy)(apriltag_family_t*);
} family_entry;

static const family_entry kFamilies[] = {
    {"tag16h5", tag16h5_create, tag16h5_destroy},
    {"tag25h9", tag25h9_create, tag25h9_destroy},
    {"tag36h10", tag36h10_create, tag36h10_destroy},
    {"tag36h11", tag36h11_create, tag36h11_destroy},
    {"tagCircle21h7", tagCircle21h7_create, tagCircle21h7_destroy},
    {"tagStandard41h12", tagStandard41h12_create, tagStandard41h12_destroy},
};
#define FAMILY_COUNT (int)(sizeof(kFamilies) / sizeof(kFamilies[0]))

static const family_entry* find_family(const char* name) {
    int i;
    if (name == NULL) {
        return NULL;
    }
    for (i = 0; i < FAMILY_COUNT; ++i) {
        if (strcmp(kFamilies[i].name, name) == 0) {
            return &kFamilies[i];
        }
    }
    return NULL;
}

struct nav_apriltag_detector {
    apriltag_detector_t* td;
    apriltag_family_t*   families[FAMILY_COUNT];
    const family_entry*  entries[FAMILY_COUNT];
    int                  count;
};

nav_apriltag_detector* nav_apriltag_create(double quad_decimate, double quad_sigma, int nthreads,
                                           int refine_edges, double decode_sharpening) {
    nav_apriltag_detector* d = (nav_apriltag_detector*)calloc(1, sizeof(nav_apriltag_detector));
    if (d == NULL) {
        return NULL;
    }
    d->td = apriltag_detector_create();
    if (d->td == NULL) {
        free(d);
        return NULL;
    }
    d->td->quad_decimate     = (float)quad_decimate;
    d->td->quad_sigma        = (float)quad_sigma;
    d->td->nthreads          = nthreads;
    d->td->refine_edges      = refine_edges != 0;
    d->td->decode_sharpening = decode_sharpening;
    d->td->debug             = false;
    return d;
}

void nav_apriltag_destroy(nav_apriltag_detector* d) {
    int i;
    if (d == NULL) {
        return;
    }
    apriltag_detector_clear_families(d->td);
    apriltag_detector_destroy(d->td);
    for (i = 0; i < d->count; ++i) {
        d->entries[i]->destroy(d->families[i]);
    }
    free(d);
}

int nav_apriltag_add_family(nav_apriltag_detector* d, const char* family) {
    const family_entry* entry = find_family(family);
    int                 i;
    if (entry == NULL) {
        return -1;
    }
    for (i = 0; i < d->count; ++i) {
        if (d->entries[i] == entry) {
            return -2;
        }
    }
    d->entries[d->count]  = entry;
    d->families[d->count] = entry->create();
    apriltag_detector_add_family(d->td, d->families[d->count]);
    ++d->count;
    return 0;
}

int nav_apriltag_detect(nav_apriltag_detector* d, int width, int height, int stride,
                        uint8_t* buffer, nav_apriltag_sink sink, void* user) {
    image_u8_t im = {width, height, stride, buffer};
    zarray_t*  detections;
    int        i, n, c;
    if (d == NULL || buffer == NULL || width <= 0 || height <= 0 || stride < width) {
        return -1;
    }
    detections = apriltag_detector_detect(d->td, &im);
    if (detections == NULL) {
        return -1;
    }
    n = zarray_size(detections);
    for (i = 0; i < n; ++i) {
        apriltag_detection_t*  det = NULL;
        nav_apriltag_detection out;
        zarray_get(detections, i, &det);
        out.family          = det->family->name;
        out.id              = det->id;
        out.hamming         = det->hamming;
        out.decision_margin = det->decision_margin;
        for (c = 0; c < 4; ++c) {
            out.corners[c][0] = det->p[c][0];
            out.corners[c][1] = det->p[c][1];
        }
        out.center[0] = det->c[0];
        out.center[1] = det->c[1];
        if (sink != NULL) {
            sink(user, &out);
        }
    }
    apriltag_detections_destroy(detections);
    return n;
}

int nav_apriltag_pose(const nav_apriltag_pose_request* request, nav_apriltag_pose_result* out) {
    /* the ideal tag square ([-1,1], y down) onto the caller's corners */
    static const double ideal[4][2] = {{-1, 1}, {1, 1}, {1, -1}, {-1, -1}};
    zarray_t*           correspondences;
    matd_t*             H;
    apriltag_detection_t      det;
    apriltag_detection_info_t info;
    apriltag_pose_t           pose1 = {NULL, NULL};
    apriltag_pose_t           pose2 = {NULL, NULL};
    double                    err1 = 0.0, err2 = 0.0;
    const apriltag_pose_t*    best;
    int                       c, r;

    if (request == NULL || out == NULL || request->tagsize_m <= 0.0 || request->fx <= 0.0 ||
        request->fy <= 0.0 || request->iterations <= 0) {
        return -1;
    }
    correspondences = zarray_create(sizeof(float[4]));
    for (c = 0; c < 4; ++c) {
        float corr[4];
        corr[0] = (float)ideal[c][0];
        corr[1] = (float)ideal[c][1];
        corr[2] = (float)request->corners[c][0];
        corr[3] = (float)request->corners[c][1];
        zarray_add(correspondences, corr);
    }
    H = homography_compute(correspondences, HOMOGRAPHY_COMPUTE_FLAG_SVD);
    zarray_destroy(correspondences);
    if (H == NULL) {
        return -1;
    }

    memset(&det, 0, sizeof(det));
    det.H = H;
    for (c = 0; c < 4; ++c) {
        det.p[c][0] = request->corners[c][0];
        det.p[c][1] = request->corners[c][1];
    }
    homography_project(H, 0, 0, &det.c[0], &det.c[1]);

    info.det     = &det;
    info.tagsize = request->tagsize_m;
    info.fx      = request->fx;
    info.fy      = request->fy;
    info.cx      = request->cx;
    info.cy      = request->cy;
    estimate_tag_pose_orthogonal_iteration(&info, &err1, &pose1, &err2, &pose2,
                                           request->iterations);
    matd_destroy(H);
    if (pose1.R == NULL || pose1.t == NULL) {
        if (pose2.R != NULL) {
            matd_destroy(pose2.R);
            if (pose2.t != NULL) {
                matd_destroy(pose2.t);
            }
        }
        return -1;
    }
    best = (pose2.R != NULL && err2 < err1) ? &pose2 : &pose1;
    for (r = 0; r < 3; ++r) {
        for (c = 0; c < 3; ++c) {
            out->R[r * 3 + c] = MATD_EL(best->R, r, c);
        }
        out->t[r] = MATD_EL(best->t, r, 0);
    }
    out->err1       = err1;
    out->err2       = pose2.R != NULL ? err2 : -1.0;
    out->has_second = pose2.R != NULL;

    matd_destroy(pose1.R);
    matd_destroy(pose1.t);
    if (pose2.R != NULL) {
        matd_destroy(pose2.R);
        if (pose2.t != NULL) {
            matd_destroy(pose2.t); /* only assigned when a second minimum exists */
        }
    }
    return 0;
}

int nav_apriltag_family_geometry(const char* family, int* width_at_border, int* total_width) {
    const family_entry* entry = find_family(family);
    apriltag_family_t*  fam;
    if (entry == NULL) {
        return -1;
    }
    fam              = entry->create();
    *width_at_border = fam->width_at_border;
    *total_width     = fam->total_width;
    entry->destroy(fam);
    return 0;
}

int nav_apriltag_render(const char* family, int id, uint8_t* cells, int capacity) {
    const family_entry* entry = find_family(family);
    apriltag_family_t*  fam;
    image_u8_t*         im;
    int                 x, y, width;
    if (entry == NULL) {
        return -1;
    }
    fam = entry->create();
    if (id < 0 || (uint32_t)id >= fam->ncodes) {
        entry->destroy(fam);
        return -2;
    }
    im    = apriltag_to_image(fam, (uint32_t)id);
    width = im->width;
    if (width * width > capacity) {
        image_u8_destroy(im);
        entry->destroy(fam);
        return 0;
    }
    for (y = 0; y < width; ++y) {
        for (x = 0; x < width; ++x) {
            cells[y * width + x] = im->buf[y * im->stride + x];
        }
    }
    image_u8_destroy(im);
    entry->destroy(fam);
    return width;
}

const char* nav_apriltag_supported_family(int index) {
    if (index < 0 || index >= FAMILY_COUNT) {
        return NULL;
    }
    return kFamilies[index].name;
}
