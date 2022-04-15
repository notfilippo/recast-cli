#pragma once

#include <string>

extern int build(std::string &filename, float cellSize, float cellHeight, float agentHeight, float agentRadius,
                 float agentMaxClimp, float agentMaxSlope, int regionMinSize, int regionMergeSize, int edgeMaxLen,
                 float edgeMaxError, int vertsPerPoly, int detailSampleDist, int detailSampleMaxError);