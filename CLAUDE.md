# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

WAVE is a high-performance WebAssembly-based voxel game engine inspired by noa-engine. It combines TypeScript for game logic with C++/WebAssembly for performance-critical operations like chunk management, meshing, and lighting. The engine features procedural world generation, greedy meshing, a two-stage lighting system, A* pathfinding, and level-of-detail rendering.

## Build Commands

### TypeScript Compilation
```bash
./scripts/build
# Or directly:
node node_modules/typescript/bin/tsc
```
This compiles TypeScript source files from `src/` to JavaScript in `target/`.

### WebAssembly Compilation
```bash
./scripts/emcc
```
This compiles C++ source files from `wasm/` to WebAssembly (`core.wasm` and `core.js`). Requires Emscripten SDK.

**Important**: The `scripts/emcc` script uses macOS-specific `sed` syntax. On Linux, remove the `''` argument from the `sed -i` command.

### Development Server
```bash
python scripts/serve.py
```
Serves the application locally. Open `index.html` in a browser with WebGL 2.0 support.

### Complete Build Workflow
```bash
./scripts/build && ./scripts/emcc
```

## Architecture

### Language Boundaries

**TypeScript Layer** (`src/`):
- Game loop and rendering coordination
- Entity Component System (ECS)
- Input handling and camera controls
- WebGL shader management and rendering pipeline
- High-level game logic

**C++/WebAssembly Layer** (`wasm/`):
- Chunk storage and management (16×256×16 voxels)
- Greedy meshing algorithm
- Two-stage cellular automaton lighting
- Procedural world generation
- Level-of-detail (LOD) system

**Communication**: TypeScript calls into WASM via exported functions. WASM returns mesh data and lighting information through shared memory buffers.

### Core Data Flow

1. **World Generation**: TypeScript requests chunk at (cx, cz) → WASM generates terrain using Perlin noise → returns run-length encoded column data
2. **Meshing**: WASM performs greedy meshing → produces quad geometry with AO → uploads to JavaScript-side buffers
3. **Lighting**: WASM runs 2-stage lighting (local chunk, then cross-chunk) → generates 3D light texture → JavaScript uploads to WebGL
4. **Rendering**: TypeScript coordinates WebGL draw calls using meshes and lighting from WASM

### Entity Component System

Located in `src/ecs.ts`. Components are stored in sparse arrays (`ComponentStore<T>`) with O(1) access by entity ID. Key components defined in `src/main.ts`:
- **Physics**: Position, velocity, bounding box, gravity
- **Movement**: Walking speed, sprint multiplier, jump height
- **Pathing**: A* pathfinding state for NPCs

Systems update components each frame. Physics uses swept AABB collision detection (`src/sweep.ts`) with sub-voxel precision.

## Critical Algorithms

### Greedy Meshing (`wasm/mesher.cpp`, `src/mesher.ts`)

Reduces polygon count by 80-95% by merging adjacent voxel faces into larger quads. The algorithm sweeps through each dimension, builds a 2D face mask, then greedily expands rectangles. **Y-axis privileged**: chunks are 16×16 in x/z but 256 in y, so the algorithm prioritizes vertical coherence for cache locality.

### Two-Stage Lighting (`wasm/engine.cpp:291-610`)

- **Stage 1**: Chunk-local cellular automaton. Iteratively propagates light from sources (sunlight, torches) with decay until convergence (typically 2-3 iterations).
- **Stage 2**: Loads 3×3 chunk neighborhood and propagates edge lighting across chunk boundaries. Only stores deltas from stage 1 for memory efficiency.

When a block changes, only dirty indices are marked for recomputation, making updates 10-100× faster than full recalculation.

### Equilevels Optimization (`README.md:112-151`)

An equi-level is a y-value where all 256 blocks (16×16 horizontal) are identical. Computed in O(runs) time using run-length encoding. During meshing, consecutive equi-levels of the same block type require no geometry, skipping 80-95% of vertical columns.

### A* Pathfinding (`src/pathing.ts:430-505`)

Classic informed search with non-admissible heuristic (prefers direct paths over optimal paths for natural NPC movement). Supports jump detection (1-3 block horizontal jumps) and path simplification via line-of-sight. Costs are tuned for natural movement:
- Horizontal move: 16
- Climb up: 64 (4× horizontal)
- Fall down: 4 (0.25× horizontal)

### Auto-Stepping Physics (`src/main.ts:204-260`)

Automatically climbs obstacles ≤0.5 blocks high without jumping. Tests an elevated sweep; if successful and moving toward the obstacle (velocity ratio >16:1), applies the step. Prevents unintended stepping when moving diagonally.

## Memory Layout & Data Structures

### Tensor3 (`src/base.ts:215-273`, `wasm/base.h:90-150`)

3D arrays with **y-axis fastest** for cache locality: `index = y + x*height + z*width*height`. This layout optimizes vertical operations (common in voxel terrain). C++ version uses bitwise operations when dimensions are powers of 2.

### Circle<T> Template (`wasm/engine.cpp:990-1098`)

Circular buffer for chunk streaming. Maintains chunks in a spiral iteration order (closest-to-center first). When recentering, reuses chunk memory from a pool to avoid allocations. Provides O(1) spatial lookup via hash table.

### Run-Length Encoding

World generation produces columns as RLE lists: `[(Block, count), ...]`. This turns cubic scaling (16×256×16) into quadratic (16×16 × runs). The engine inflates to 3D using TypedArray.fill() (fast native code).

## Rendering Pipeline

### Render Passes (`src/renderer.ts:2209-2267`)

1. **Shadow Map**: Renders scene from sun's perspective to 1024×1024 depth texture
2. **Opaque Geometry**: Terrain, decorations, entities with depth testing
3. **Transparent Geometry**: Water, glass with alpha blending (depth writes disabled)
4. **Overlays**: UI, debug visualization (no depth testing)

### VoxelShader

Vertex shader unpacks compressed quad data (4 uint32s per quad) and generates 4 vertices with:
- Position (world coordinates)
- UV coordinates (texture atlas)
- Lighting (3D light texture sample × ambient occlusion)
- Wave animation for water

Fragment shader applies texture, lighting, and exponential fog.

### Buffer Pooling (`src/renderer.ts:BufferAllocator`)

Reuses WebGL buffers to eliminate allocations. When releasing a buffer, returns it to a free list. When allocating, tries to reuse a sufficiently large buffer before creating new. Critical for stable performance.

## Common Development Tasks

### Scene Modding with Duplicate Files (Recommended Approach)

**Philosophy**: Keep the original engine pristine, modify duplicates for custom content.

**Files**:
- `src/main.ts` - **Original engine scene (DO NOT MODIFY)**
- `src/main-duplicate.ts` - **Your modded scene (edit freely)**
- `index.html` - Switch between versions by changing import

**Workflow**:
1. Edit `src/main-duplicate.ts` with your scene modifications
2. Build: `node node_modules/typescript/bin/tsc`
3. Update `index.html` line 11:
   ```html
   <script type="module" src="target/src/main-duplicate.js"></script>
   ```
4. Refresh browser to see changes

**Benefits**:
- Engine updates don't break your mods
- Easy to compare original vs modified
- Can maintain multiple mod versions
- Clear separation of engine vs content

**Example Modifications**:
- Terrain mods: Flat platforms, hills, ponds, paths (`createFlatPlatform()`, etc.)
- Entity spawns: Additional NPCs, custom starting positions
- Block materials: Change textures, colors, properties
- Structures: Programmatically placed buildings, landmarks

See `SCENE_DOCUMENTATION.md`, `MODDING_GUIDE.md`, and `TERRAIN_MODS_REFERENCE.md` for detailed examples.

### Adding a Block Type

1. Define enum in `wasm/base.h`: `enum class Block : uint8_t { MyBlock = 100 };`
2. Configure in `src/main.ts` blocks object with `id`, `opaque`, `solid`, `light`, `textures`
3. Add texture to `images/` directory (16×16 or 32×32 PNG)
4. Register via `env.helper.registerBlock()`
5. Rebuild: `./scripts/build && ./scripts/emcc`

### Modifying Terrain Generation (WASM Layer)

Edit `wasm/worldgen.cpp`:
- **Heightmap**: Adjust `getHeight()` noise octaves
- **Biomes**: Modify `getBlockForHeight()` based on temperature/moisture noise
- **Caves**: Change `shouldCaveCarve()` threshold (higher = more caves)
- Rebuild WASM only: `./scripts/emcc`

**Note**: For post-generation terrain modifications (structures, landmarks), use the scene modding approach above instead of modifying worldgen.

### Debugging WASM

Compile with debug symbols (already enabled: `-g2` in `scripts/emcc`). Use browser DevTools:
- Chrome: `chrome://inspect` → WebAssembly debugging
- Firefox: `about:debugging` → enable WASM source maps

### Performance Profiling

Check `src/engine.ts` performance tracking:
```typescript
console.log('Frame time:', performance.frame_time, 'ms');
console.log('Render time:', performance.render_time, 'ms');
console.log('Remesh time:', performance.remesh_time, 'ms');
```

Browser DevTools Performance tab can profile JavaScript and WASM together.

## Key Files Reference

### Core Engine
- `src/engine.ts`: Main game loop, camera, WASM interface (~600 lines)
- `src/main.ts`: **Original scene (DO NOT MODIFY)** - Game logic, components, block modification (~1200 lines)
- `src/main-duplicate.ts`: **Modded scene** - Your custom scene modifications (~1200+ lines)
- `src/renderer.ts`: Complete WebGL pipeline (~2200 lines)
- `src/mesher.ts`: Client-side meshing utilities (~400 lines)
- `src/pathing.ts`: A* pathfinding implementation (~580 lines)
- `src/sweep.ts`: Swept AABB collision (~100 lines)
- `src/base.ts`: Math utilities, Tensor classes (~600 lines)
- `src/ecs.ts`: Entity Component System (~142 lines)
- `wasm/engine.cpp`: World, Chunk, lighting (~1558 lines)
- `wasm/mesher.cpp`: Greedy meshing, AO (~700 lines)
- `wasm/worldgen.cpp`: Procedural generation (~400 lines)
- `wasm/base.h`: Shared C++ data structures (~200 lines)

### Documentation
- `CLAUDE.md`: This file - guidance for Claude Code
- `WAVE_ENGINE_DOCUMENTATION.md`: Complete technical reference
- `SCENE_DOCUMENTATION.md`: Line-by-line scene explanation
- `MODDING_GUIDE.md`: Safe modding workflow and examples
- `TERRAIN_MODS_REFERENCE.md`: Terrain modification techniques

## Important Notes

- **No npm modules**: TypeScript is vendored in `node_modules/`. Minimal external dependencies by design.
- **Y-axis is vertical**: Chunks are 16×256×16 where y ∈ [0, 255] is height
- **Sub-voxel precision**: Physics uses 4096 subpixels per block to prevent tunneling
- **WebGL 2.0 required**: Uses 2D texture arrays for single-draw-call chunk rendering
- **Lighting requires neighbors**: Stage 2 lighting needs 3×3 chunk neighborhood loaded
- **Fixed timestep physics**: 20 ticks per second (50ms), decoupled from rendering

## Performance Characteristics

- **Target**: 60 FPS with 450+ loaded chunks
- **Meshing**: ~5ms per chunk (one-time cost)
- **Lighting Stage 1**: ~1ms per chunk
- **Lighting Stage 2**: ~2ms per chunk
- **World Gen**: ~2ms per chunk
- **Memory**: ~350KB per chunk (voxels + lighting + meshes)
- **Greedy meshing**: 80-95% polygon reduction vs naive cubes
- **Equilevels**: Skips meshing for 80-95% of uniform layers

For detailed technical documentation, see `WAVE_ENGINE_DOCUMENTATION.md`.
