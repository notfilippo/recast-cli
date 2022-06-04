# Recast CLI

Create a navigation mesh from an OBJ mesh.

## Running

### Docker

```bash
docker run --rm -v `pwd`:/input ghcr.io/notfilippo/recast-cli:master -filename=/input/mesh.obj
```

### Local build

```bash
mkdir -p build
cd build
cmake ..
```

And then `RecastCli` can simply be called with:

```bash
./RecastCli -filename=mesh.obj
```

## Options

| Option | Type | Default value | Description |
| - | - | - | - |
| `-filename` | string | "" | Mesh filename |
| `-cellSize` | double | `CFG_CELL_SIZE` | Cell size |
| `-cellHeight` | double | `CFG_CELL_HEIGHT` | Cell height |
| `-agentHeight` | double | `CFG_AGENT_HEIGHT` | Agent height |
| `-agentRadius` | double | `CFG_AGENT_RADIUS` | Agent radius |
| `-agentMaxClimp` | double | `CFG_AGENT_MAX_CLIMP` | Agent max climp |
| `-agentMaxSlope` | double | `CFG_AGENT_MAX_SLOPE` | Agent max slope |
| `-regionMinSize` | int32 | `CFG_REGION_MIN_SIZE` | Region min size |
| `-regionMergeSize` | int32 | `CFG_REGION_MERGE_SIZE` | Region merge size |
| `-edgeMaxLen` | int32 | `CFG_EDGE_MAX_LEN` | Edge max len |
| `-edgeMaxError` | double | `CFG_EDGE_MAX_ERROR` | Edge max error |
| `-vertsPerPoly` | int32 | `CFG_VERTS_PER_POLY` | Vertices per polygon |
| `-detailSampleDist` | int32 | `CFG_DETAIL_SAMPLE_DIST` | Detail sample dist |
| `-detailSampleMaxErro` | int32 | `CFG_DETAIL_SAMPLE_MAX_ERROR` | Details sample max error |

## Acknowledgements

- https://github.com/recastnavigation/recastnavigation
- https://github.com/but0n/recastCLI.js
