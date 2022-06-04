#include <gflags/gflags.h>

#include <iostream>
#include <sstream>

#include "Recast.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"

#include "build.h"

// Default setting

// Rasterization
#define CFG_CELL_SIZE 0.1
#define CFG_CELL_HEIGHT 0.2

// Agent
#define CFG_AGENT_HEIGHT 2.0
#define CFG_AGENT_RADIUS 0.25
#define CFG_AGENT_MAX_CLIMP 0.9
#define CFG_AGENT_MAX_SLOPE 45

// Region
#define CFG_REGION_MIN_SIZE 8
#define CFG_REGION_MERGE_SIZE 20

// Polygonization
#define CFG_EDGE_MAX_LEN 12
#define CFG_EDGE_MAX_ERROR 1.3
#define CFG_VERTS_PER_POLY 6

// Detail Mesh
#define CFG_DETAIL_SAMPLE_DIST 6
#define CFG_DETAIL_SAMPLE_MAX_ERROR 1

DEFINE_string(filename, "", "Mesh filename");

DEFINE_double(cellSize, CFG_CELL_SIZE, "Cell size");
DEFINE_double(cellHeight, CFG_CELL_HEIGHT, "Cell height");

DEFINE_double(agentHeight, CFG_AGENT_HEIGHT, "Agent height");
DEFINE_double(agentRadius, CFG_AGENT_RADIUS, "Agent radius");
DEFINE_double(agentMaxClimp, CFG_AGENT_MAX_CLIMP, "Agent max climp");
DEFINE_double(agentMaxSlope, CFG_AGENT_MAX_SLOPE, "Agent max slope");

DEFINE_int32(regionMinSize, CFG_REGION_MIN_SIZE, "Region min size");
DEFINE_int32(regionMergeSize, CFG_REGION_MERGE_SIZE, "Region merge size");

DEFINE_int32(edgeMaxLen, CFG_EDGE_MAX_LEN, "Edge max len");
DEFINE_double(edgeMaxError, CFG_EDGE_MAX_ERROR, "Edge max error");

DEFINE_int32(vertsPerPoly, CFG_VERTS_PER_POLY, "Vertices per polygon");

DEFINE_int32(detailSampleDist, CFG_DETAIL_SAMPLE_DIST, "Detail sample dist");
DEFINE_int32(detailSampleMaxErro, CFG_DETAIL_SAMPLE_MAX_ERROR, "Details sample max error");

int main(int argc, char* argv[])
{
    std::stringstream usage;
    usage << "usage: " << argv[0] << " -filename=[FILENAME]";

    gflags::SetUsageMessage(usage.str());
    gflags::ParseCommandLineFlags(&argc, &argv, true);

    if (FLAGS_filename.empty()) {
        std::cout << usage.str() << std::endl;
        return 1;
    }

    rcPolyMesh* polyMesh = rcAllocPolyMesh();
    rcPolyMeshDetail* polyMeshDetail = rcAllocPolyMeshDetail();
    if (!build(FLAGS_filename, (float)FLAGS_cellSize, (float)FLAGS_cellHeight, (float)FLAGS_agentHeight,
            (float)FLAGS_agentRadius, (float)FLAGS_agentMaxClimp, (float)FLAGS_agentMaxSlope, FLAGS_regionMinSize,
            FLAGS_regionMergeSize, FLAGS_edgeMaxLen, (float)FLAGS_edgeMaxError, FLAGS_vertsPerPoly,
            FLAGS_detailSampleDist, FLAGS_detailSampleMaxErro, polyMesh, polyMeshDetail)) {
        std::cout << "Build failed" << std::endl;
    }

    const float cs = polyMesh->cs;
    const float ch = polyMesh->ch;

    for (int i = 0; i < polyMesh->nverts; i++) {
        float x = polyMesh->bmin[0] + (float)polyMesh->verts[i * 3 + 0] * cs;
        float y = polyMesh->bmin[1] + (float)polyMesh->verts[i * 3 + 1] * ch;
        float z = polyMesh->bmin[2] + (float)polyMesh->verts[i * 3 + 2] * cs;
        printf("v %f %f %f\r\n", x, y, z);
    }
    printf("\r\n");

    for (int i = 0; i < polyMesh->npolys; i++) {
        const unsigned short* poly = &polyMesh->polys[i * 2 * polyMesh->nvp];
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