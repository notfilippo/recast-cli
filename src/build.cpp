#include "build.h"

#include <cstdio>
#include <cstring>

#include "InputGeom.h"
#include "Recast.h"
#include "cmath"

#define m_keepInterResults true
#define m_filterLowHangingObstacles true
#define m_filterLedgeSpans true
#define m_filterWalkableLowHeightSpans true

enum SamplePartitionType {
    SAMPLE_PARTITION_WATERSHED, SAMPLE_PARTITION_MONOTONE, SAMPLE_PARTITION_LAYERS,
};

int build(std::string &filename, float cellSize, float cellHeight, float agentHeight, float agentRadius,
          float agentMaxClimp, float agentMaxSlope, int regionMinSize, int regionMergeSize, int edgeMaxLen,
          float edgeMaxError, int vertsPerPoly, int detailSampleDist, int detailSampleMaxError) {

    rcConfig config{};

    auto ctx = new rcContext();
    auto geom = new InputGeom();

    rcHeightfield *heightfield;
    unsigned char *triareas;

    rcCompactHeightfield *compactHeightfield;

    rcContourSet *contourSet;

    rcPolyMesh *polyMesh;
    rcPolyMeshDetail *polyMeshDetail;

    if (!geom->load(ctx, filename)) {
        ctx->log(RC_LOG_ERROR, "Cannot load file: %s", filename.c_str());
        return -1;
    }

    const float *bmin = geom->getNavMeshBoundsMin();
    const float *bmax = geom->getNavMeshBoundsMax();

    const float *verts = geom->getMesh()->getVerts();
    const int nverts = geom->getMesh()->getVertCount();

    const int *tris = geom->getMesh()->getTris();
    const int ntris = geom->getMesh()->getTriCount();

    if (!geom->getMesh()) {
        ctx->log(RC_LOG_ERROR, "buildTiledNavigation: Out of memory 'm_chunkyMesh'.");
        return false;
    }

    memset(&config, 0, sizeof(config));

    config.cs = cellSize;
    config.ch = cellHeight;

    config.walkableHeight = (int) ceilf(agentHeight / config.ch);
    config.walkableRadius = (int) ceilf(agentRadius / config.cs);
    config.walkableClimb = (int) floorf(agentMaxClimp / config.ch);
    config.walkableSlopeAngle = agentMaxSlope;

    config.maxEdgeLen = (int) ((float) edgeMaxLen / config.cs);
    config.maxSimplificationError = (float) edgeMaxError;
    config.maxVertsPerPoly = (int) vertsPerPoly;

    config.minRegionArea = (int) rcSqr(regionMinSize);
    config.mergeRegionArea = (int) rcSqr(regionMergeSize);

    config.detailSampleDist = (float) detailSampleDist < 0.9f ? 0 : config.cs * (float) detailSampleDist;
    config.detailSampleMaxError = (float) config.ch * (float) detailSampleMaxError;

    rcVcopy(config.bmin, bmin);
    rcVcopy(config.bmax, bmax);

    rcCalcGridSize(config.bmin, config.bmax, config.cs, &config.width, &config.height);

    ctx->resetTimers();

    ctx->startTimer(RC_TIMER_TOTAL);

    heightfield = rcAllocHeightfield();
    if (!heightfield) {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'solid'.");
        return false;
    }

    if (!rcCreateHeightfield(ctx, *heightfield, config.width, config.height, config.bmin, config.bmax, config.cs, config.ch)) {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create solid heightfield.");
        return false;
    }

    triareas = new unsigned char[ntris];

    memset(triareas, 0, ntris * sizeof(unsigned char));

    rcMarkWalkableTriangles(ctx, config.walkableSlopeAngle, verts, nverts, tris, ntris, triareas);

    if (!rcRasterizeTriangles(ctx, verts, nverts, tris, triareas, ntris, *heightfield, config.walkableClimb)) {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Could not rasterize triangles.");
        return false;
    }

    if (!m_keepInterResults) {
        delete[] triareas;
        triareas = 0;
    }

    if (m_filterLowHangingObstacles)
        rcFilterLowHangingWalkableObstacles(ctx, config.walkableClimb, *heightfield);

    if (m_filterLedgeSpans)
        rcFilterLedgeSpans(ctx, config.walkableHeight, config.walkableClimb, *heightfield);

    if (m_filterWalkableLowHeightSpans)
        rcFilterWalkableLowHeightSpans(ctx, config.walkableHeight, *heightfield);

    compactHeightfield = rcAllocCompactHeightfield();
    if (!compactHeightfield) {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'chf'.");
        return false;
    }

    if (!rcBuildCompactHeightfield(ctx, config.walkableHeight, config.walkableClimb, *heightfield, *compactHeightfield)) {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build compact data.");
        return false;
    }

    if (!m_keepInterResults) {
        rcFreeHeightField(heightfield);
        heightfield = 0;
    }

    if (!rcErodeWalkableArea(ctx, config.walkableRadius, *compactHeightfield)) {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Could not erode.");
        return false;
    }

    const ConvexVolume *vols = geom->getConvexVolumes();
    for (int i = 0; i < geom->getConvexVolumeCount(); ++i)
        rcMarkConvexPolyArea(ctx, vols[i].verts, vols[i].nverts, vols[i].hmin, vols[i].hmax,
                             (unsigned char) vols[i].area, *compactHeightfield);

    if (!rcBuildDistanceField(ctx, *compactHeightfield)) {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build distance field.");
        return false;
    }

    if (!rcBuildRegions(ctx, *compactHeightfield, 0, config.minRegionArea, config.mergeRegionArea)) {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build watershed regions.");
        return false;
    }

    contourSet = rcAllocContourSet();
    if (!contourSet) {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'cset'.");
        return false;
    }
    if (!rcBuildContours(ctx, *compactHeightfield, config.maxSimplificationError, config.maxEdgeLen, *contourSet)) {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Could not create contours.");
        return false;
    }

    polyMesh = rcAllocPolyMesh();
    if (!polyMesh) {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmesh'.");
        return false;
    }
    if (!rcBuildPolyMesh(ctx, *contourSet, config.maxVertsPerPoly, *polyMesh)) {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Could not triangulate contours.");
        return false;
    }

    polyMeshDetail = rcAllocPolyMeshDetail();
    if (!polyMeshDetail) {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Out of memory 'pmdtl'.");
        return false;
    }

    if (!rcBuildPolyMeshDetail(ctx, *polyMesh, *compactHeightfield, config.detailSampleDist, config.detailSampleMaxError, *polyMeshDetail)) {
        ctx->log(RC_LOG_ERROR, "buildNavigation: Could not build detail mesh.");
        return false;
    }

    if (!m_keepInterResults) {
        rcFreeCompactHeightfield(compactHeightfield);
        compactHeightfield = nullptr;
        rcFreeContourSet(contourSet);
        contourSet = nullptr;
    }

    const float cs = polyMesh->cs;
    const float ch = polyMesh->ch;

    for (int i = 0; i < polyMesh->nverts; i++) {
        float x = polyMesh->bmin[0] + (float) polyMesh->verts[i * 3 + 0] * cs;
        float y = polyMesh->bmin[1] + (float) polyMesh->verts[i * 3 + 1] * ch;
        float z = polyMesh->bmin[2] + (float) polyMesh->verts[i * 3 + 2] * cs;
        printf("v %f %f %f\r\n", x, y, z);
    }
    printf("\r\n");

    for (int i = 0; i < polyMesh->npolys; i++) {
        const unsigned short *poly = &polyMesh->polys[i * 2 * polyMesh->nvp];
        printf("f ");
        for (int v = 0; v < polyMesh->nvp; v++) {
            if (poly[v] == RC_MESH_NULL_IDX) {
                break;
            } else {
                printf("%d ", poly[v] + 1);
            }
        }
        printf("\r\n");
    }

    return 0;
}