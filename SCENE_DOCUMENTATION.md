# WAVE Engine Scene Documentation

This document provides a line-by-line explanation of the scene setup in `main.ts`, specifically the `main()` function (lines 1202-1268) which initializes the game world.

## Table of Contents
- [Scene Overview](#scene-overview)
- [Line-by-Line Breakdown](#line-by-line-breakdown)
- [Entities](#entities)
- [Textures and Materials](#textures-and-materials)
- [Blocks](#blocks)
- [Modding Guide](#modding-guide)

---

## Scene Overview

The `main()` function is the entry point that sets up the entire game world. It:
1. Creates the environment container
2. Spawns player and follower entities
3. Defines texture helper functions
4. Registers materials (textures with properties)
5. Registers block types
6. Refreshes the world to apply all settings

**Location**: `src/main.ts:1202-1268`

---

## Line-by-Line Breakdown

### Environment Initialization

```typescript
1202: const main = () => {
1203:   const env = new TypedEnv('container');
```

**What it does**:
- Creates a new `TypedEnv` instance which is the central game environment
- `'container'` is the HTML element ID where the canvas will be attached
- `TypedEnv` initializes all core systems: ECS, renderer, physics, etc.

**Where it's defined**: `src/main.ts:45-75`

---

### Entity Setup

```typescript
1205:   const size = 1.5;
1206:   const [x, z] = [1, 1];
```

**What it does**:
- `size`: Visual size of entity sprites (1.5 blocks tall when rendered)
- `[x, z]`: Starting spawn position in world coordinates (x=1, z=1)
- Y-coordinate is auto-calculated by `safeHeight()` to place entities on terrain

**Why [1, 1]?**: Close to world origin (0, 0) but not exactly on it

---

### Player Entity

```typescript
1207:   const player = addEntity(env, 'player', size, x, z, 1.5, 0.75, 8, 4, 10, 7.5);
1208:   env.inputs.add(player);
1209:   env.target.add(player);
```

**addEntity() parameters**:
1. `env`: Game environment
2. `'player'`: Image file (`images/player.png`)
3. `size`: 1.5 (sprite render size)
4. `x, z`: 1, 1 (spawn position)
5. `h`: 1.5 (collision height in blocks)
6. `w`: 0.75 (collision width in blocks)
7. `maxSpeed`: 8 (blocks per second)
8. `moveForceFactor`: 4 (multiplier for movement force)
9. `jumpForce`: 10 (continuous upward force while holding jump)
10. `jumpImpulse`: 7.5 (initial jump velocity)

**Components added**:
- Line 1208: `inputs` - Processes keyboard/mouse input
- Line 1209: `target` - Makes camera follow this entity

**Function reference**: `addEntity()` at `src/main.ts:1170-1200`

---

### Follower Entity (NPC)

```typescript
1211:   const follower = addEntity(env, 'follower', size, x, z, 0.75, 0.75, 12, 8, 15, 10);
1212:   env.meshes.getX(follower).heading = 0;
1213:   env.pathing.add(follower);
```

**Differences from player**:
- `'follower'`: Uses `images/follower.png`
- `h`: 0.75 (shorter than player - 3/4 block tall)
- `w`: 0.75 (same width as player)
- `maxSpeed`: 12 (50% faster than player)
- `moveForceFactor`: 8 (2× player, more responsive)
- `jumpForce`: 15 (50% stronger jumps)
- `jumpImpulse`: 10 (33% stronger initial velocity)

**Line 1212**: Sets initial facing direction (0 radians = north)

**Line 1213**: Adds `pathing` component for A* pathfinding AI

**Note**: Follower emits light (level 14) - see line 1194 in `addEntity()`

---

### Texture Helper Functions

```typescript
1215:   const white: Color = [1, 1, 1, 1];
1216:   const texture = (x: int, y: int, alphaTest: boolean = false,
1217:                    color: Color = white, sparkle: boolean = false): Texture => {
1218:     const url = 'images/frlg.png';
1219:     return {alphaTest, color, sparkle, url, x, y, w: 16, h: 16};
1220:   };
```

**What it does**:
- Creates a helper function to reference textures from a sprite atlas
- `white`: Default tint color (no tint)
- `texture(x, y)`: Returns texture at grid position (x, y) in atlas
- Atlas file: `images/frlg.png` (Fire Red / Leaf Green tileset)
- Grid size: 16×16 (each texture is 16 pixels square)

**Parameters**:
- `x, y`: Grid coordinates in atlas (0-based)
- `alphaTest`: If true, discard fully transparent pixels (for vegetation)
- `color`: RGBA tint applied to texture `[R, G, B, A]` (0-1 range)
- `sparkle`: If true, adds animated sparkle effect (for water)

**Example**: `texture(0, 0)` returns the texture at top-left of atlas

---

### Block Mesh Helper

```typescript
1222:   const block = (x: int, y: int) => {
1223:     const url = 'images/frlg.png';
1224:     const frame = int(x + 16 * y);
1225:     return env.renderer.addInstancedMesh(frame, {url, x: 16, y: 16});
1226:   };
```

**What it does**:
- Creates instanced mesh for decorative blocks (bushes, rocks, fungi)
- Instanced meshes are more efficient for many copies of the same model
- `frame`: Calculates linear index from 2D grid position
  - Formula: `frame = x + (y × atlas_width)`
  - Example: `(2, 1)` → frame 18 (skip first row of 16, then +2)

**Use case**: Non-solid decorations (see lines 1250, 1252, 1254)

---

### Materials Registration

```typescript
1228:   const registry = env.registry;
1229:   registry.addMaterial(
1230:       'blue', texture(12, 0, false, [0.1, 0.1, 0.4, 0.6]), true);
1231:   registry.addMaterial(
1232:       'water', texture(11, 0, false, [1, 1, 1, 0.8], true), true);
```

**What it does**:
- Registers special materials with custom properties
- `registry`: Block/material registry from environment

**Blue material (line 1230)**:
- Name: `'blue'`
- Texture: Grid position (12, 0)
- Color tint: Dark blue `[0.1, 0.1, 0.4, 0.6]` with 60% opacity
- Fluid: `true` (last parameter - entities can swim through)

**Water material (line 1232)**:
- Name: `'water'`
- Texture: Grid position (11, 0)
- Color tint: White with 80% opacity (semi-transparent)
- Sparkle: `true` (adds animated water shimmer)
- Fluid: `true`

**Note**: Materials are texture + rendering properties. Blocks reference materials.

---

### Standard Textures Array

```typescript
1233:   const textures: [string, int, int][] = [
1234:     ['bedrock', 6, 0],
1235:     ['dirt', 2, 0],
1236:     ['grass', 0, 0],
1237:     ['grass-side', 3, 0],
1238:     ['stone', 1, 0],
1239:     ['sand', 4, 0],
1240:     ['snow', 5, 0],
1241:     ['trunk', 8, 0],
1242:     ['trunk-side', 7, 0],
1243:   ];
```

**What it does**:
- Defines array of texture definitions: `[name, x, y]`
- Each entry maps a material name to atlas coordinates

**Texture reference**:
| Name | Position | Use |
|------|----------|-----|
| bedrock | (6, 0) | Unbreakable bottom layer |
| dirt | (2, 0) | Underground, under grass |
| grass | (0, 0) | Grass top surface |
| grass-side | (3, 0) | Grass block sides |
| stone | (1, 0) | Underground stone |
| sand | (4, 0) | Beaches, deserts |
| snow | (5, 0) | Snow-covered blocks |
| trunk | (8, 0) | Tree trunk top/bottom |
| trunk-side | (7, 0) | Tree trunk sides |

---

### Material Registration Loop

```typescript
1244:   for (const [name, x, y] of textures) {
1245:     registry.addMaterial(name, texture(x, y));
1246:   }
```

**What it does**:
- Loops through `textures` array
- Registers each material with default properties
- Uses `texture(x, y)` helper (no tint, no sparkle, opaque)

**Result**: All standard block materials are now registered

---

### Block Registration

```typescript
1248:   const blocks = {
1249:     bedrock: registry.addBlock(['bedrock'], true),
1250:     bush:    registry.addBlockMesh(block(10, 0), false),
1251:     dirt:    registry.addBlock(['dirt'], true),
1252:     fungi:   registry.addBlockMesh(block(13, 0), false, 9),
1253:     grass:   registry.addBlock(['grass', 'dirt', 'grass-side'], true),
1254:     rock:    registry.addBlockMesh(block(9, 0), true),
1255:     sand:    registry.addBlock(['sand'], true),
1256:     snow:    registry.addBlock(['snow'], true),
1257:     stone:   registry.addBlock(['stone'], true),
1258:     trunk:   registry.addBlock(['trunk', 'trunk-side'], true),
1259:     water:   registry.addBlock(['water', 'blue', 'blue'], false),
1260:   };
```

**What it does**:
- Creates block type definitions and assigns unique IDs
- Two types of blocks:
  1. **Cube blocks** (`addBlock`): Standard voxel cubes
  2. **Mesh blocks** (`addBlockMesh`): Cross-shaped decorations

---

#### Cube Blocks (addBlock)

**Function signature**: `addBlock(materials: string[], solid: boolean): BlockId`

**Parameters**:
- `materials`: Array of material names for faces
  - Length 1: All faces use same material
  - Length 2: `[top/bottom, sides]`
  - Length 3: `[top, bottom, sides]`
  - Length 6: `[+Y, -Y, +X, -X, +Z, -Z]` (rarely used)
- `solid`: Collision - `true` blocks movement, `false` passes through

**Examples**:

| Block | Materials | Solid | Notes |
|-------|-----------|-------|-------|
| bedrock | `['bedrock']` | ✓ | Same texture all faces |
| dirt | `['dirt']` | ✓ | Same texture all faces |
| grass | `['grass', 'dirt', 'grass-side']` | ✓ | Top, bottom, sides |
| sand | `['sand']` | ✓ | Beach/desert block |
| snow | `['snow']` | ✓ | Snow layer |
| stone | `['stone']` | ✓ | Underground stone |
| trunk | `['trunk', 'trunk-side']` | ✓ | Top/bottom, sides |
| water | `['water', 'blue', 'blue']` | ✗ | Top, bottom, sides - **not solid** |

**Note**: Water is **not solid** - entities can pass through and swim

---

#### Mesh Blocks (addBlockMesh)

**Function signature**: `addBlockMesh(mesh: InstancedMesh, solid: boolean, light?: number): BlockId`

**Parameters**:
- `mesh`: Instanced mesh (cross-plane billboard)
- `solid`: Collision property
- `light`: Optional light emission level (0-14)

**Examples**:

| Block | Mesh | Solid | Light | Notes |
|-------|------|-------|-------|-------|
| bush | `block(10, 0)` | ✗ | 0 | Grass decoration |
| fungi | `block(13, 0)` | ✗ | 9 | Glowing mushroom |
| rock | `block(9, 0)` | ✓ | 0 | Solid boulder |

**Visual**: Mesh blocks render as 2 crossed planes (like Minecraft tall grass)

```
Top view:
  \  /
   \/    (Two perpendicular planes forming an X)
   /\
  /  \
```

---

### Final Setup

```typescript
1262:   env.blocks = blocks;
1263:   env.refresh();
1264: };
```

**Line 1262**: Stores block registry in environment for easy access

**Line 1263**: Calls `refresh()` to:
- Generate initial chunks around spawn
- Run lighting calculations
- Build meshes
- Upload to GPU

**Line 1264**: End of `main()` function

---

### Initialization Call

```typescript
1268: init(main);
```

**What it does**:
- Calls the engine's `init()` function with `main` as callback
- `init()` sets up WebGL, loads WASM module, then calls `main()`
- Entry point for entire engine

**Location**: `src/engine.ts` (WASM initialization and game loop setup)

---

## Entities

### Entity Structure

Every entity has:
- **ID**: Unique integer identifier
- **Position**: Bounding box (x, y, z, width, height)
- **Components**: Optional systems (Physics, Movement, Pathing, Meshes, etc.)

### Player Entity

```typescript
const player = addEntity(env, 'player', size, x, z, 1.5, 0.75, 8, 4, 10, 7.5);
env.inputs.add(player);      // Keyboard/mouse control
env.target.add(player);      // Camera follows
```

**Properties**:
- Collision box: 1.5 blocks tall, 0.75 blocks wide
- Speed: 8 blocks/second
- Controlled by user input
- Camera target (3rd person view)
- No light emission

### Follower Entity

```typescript
const follower = addEntity(env, 'follower', size, x, z, 0.75, 0.75, 12, 8, 15, 10);
env.meshes.getX(follower).heading = 0;
env.pathing.add(follower);   // AI pathfinding
```

**Properties**:
- Collision box: 0.75 blocks tall, 0.75 blocks wide (cube)
- Speed: 12 blocks/second (50% faster than player)
- AI-controlled (follows player using A* pathfinding)
- Emits light (level 14, almost as bright as torch)
- More responsive movement (higher force multiplier)

---

## Textures and Materials

### Texture Atlas

**File**: `images/frlg.png`
**Layout**: 16×16 grid of 16×16 pixel textures

**Coordinate system**: (0, 0) is top-left

Example positions:
```
(0,0)  (1,0)  (2,0)  (3,0)  ...
(0,1)  (1,1)  (2,1)  (3,1)  ...
...
```

### Material Properties

**Texture**: Base image from atlas
**Color**: RGBA tint `[R, G, B, A]` (0-1 range)
**Alpha test**: Discard fully transparent pixels
**Sparkle**: Animated shimmer effect
**Fluid**: Can entities swim through?

### Special Materials

**Water**:
- Semi-transparent (80% opacity)
- Animated sparkle effect
- Fluid (can swim)
- Tinted white (no color change)

**Blue** (underwater):
- Dark blue tint
- 60% opacity
- Fluid (can swim)
- Used for water bottom/sides

---

## Blocks

### Block Types

1. **Cube blocks**: Standard voxels (most blocks)
2. **Mesh blocks**: Decorative cross-planes (bushes, fungi, rocks)

### Cube Block Face Mapping

**1 material**: All faces same
```
  [material]
  → All 6 faces
```

**2 materials**: Top/bottom vs sides
```
  [top/bottom, sides]
  → +Y/-Y: material[0]
  → +X/-X/+Z/-Z: material[1]
```

**3 materials**: Top, bottom, sides
```
  [top, bottom, sides]
  → +Y: material[0]
  → -Y: material[1]
  → +X/-X/+Z/-Z: material[2]
```

### Solid vs Non-Solid

**Solid blocks**:
- Block entity movement (collision)
- Examples: stone, dirt, grass, bedrock

**Non-solid blocks**:
- Entities pass through
- Examples: water, bushes, fungi

**Note**: Non-solid ≠ transparent. Water is non-solid but visible.

---

## Modding Guide

### How to Modify the Scene

Use `main-duplicate.ts` as your mod file:

1. **Copy `main.ts` to `main-duplicate.ts`** (already done)
2. **Modify the duplicate**, never touch `main.ts`
3. **Import duplicate in `index.html`**:
   ```html
   <script type="module" src="main-duplicate.js"></script>
   ```
4. **Document changes** (see below)

---

### Common Modifications

#### Change Starting Position

```typescript
// Original:
const [x, z] = [1, 1];

// Modified:
const [x, z] = [100, 200];  // Spawn far from origin
```

#### Add More Entities

```typescript
// After follower, add:
const guard = addEntity(env, 'follower', size, 10, 10, 1.5, 0.75, 8, 4, 10, 7.5);
env.pathing.add(guard);  // AI-controlled
```

#### Change Entity Properties

```typescript
// Make player faster:
const player = addEntity(env, 'player', size, x, z, 1.5, 0.75, 15, 6, 10, 7.5);
//                                                            ^^  ^ (faster speed & force)
```

#### Add New Block Type

1. **Add to textures array**:
   ```typescript
   const textures: [string, int, int][] = [
     ['bedrock', 6, 0],
     // ... existing ...
     ['custom-block', 14, 3],  // New texture at (14, 3)
   ];
   ```

2. **Register block**:
   ```typescript
   const blocks = {
     // ... existing ...
     custom: registry.addBlock(['custom-block'], true),
   };
   ```

#### Modify Water Properties

```typescript
// Original water:
registry.addMaterial('water', texture(11, 0, false, [1, 1, 1, 0.8], true), true);

// Green toxic water:
registry.addMaterial('water', texture(11, 0, false, [0.2, 1, 0.2, 0.6], true), true);
//                                                    ^^^^^^^^^^^^^^^^ (green tint, more transparent)
```

#### Add Light-Emitting Blocks

```typescript
// Glowing block (like fungi but solid):
torch: registry.addBlock(['torch-texture'], true, 14),  // Light level 14
```

**Note**: `addBlock` accepts optional 3rd parameter for light emission

---

### Documentation Template

When modifying `main-duplicate.ts`, add a comment block at the top:

```typescript
//////////////////////////////////////////////////////////////////////////////
// MODIFICATIONS TO ORIGINAL SCENE
// Date: YYYY-MM-DD
// Author: Your Name
//
// Changes:
// - Line XXX: Changed starting position from [1, 1] to [100, 200]
// - Line YYY: Added 3 additional follower entities
// - Line ZZZ: Modified water color to green tint
// - Line AAA: Increased player speed from 8 to 15
//
// Reasoning:
// - Testing spawn system at various world coordinates
// - Evaluating pathfinding with multiple agents
// - Creating toxic water biome visual
// - Playtesting high-speed movement mechanics
//////////////////////////////////////////////////////////////////////////////
```

---

## Summary

The `main()` function is the scene definition. It:
1. ✓ Creates the game environment
2. ✓ Spawns player (user-controlled) + follower (AI)
3. ✓ Defines textures from sprite atlas
4. ✓ Registers materials (textures + properties)
5. ✓ Registers blocks (cube + mesh types)
6. ✓ Refreshes world to apply all settings

**Key insight**: This is declarative scene setup, not procedural generation. The world terrain itself is generated by WASM (see `wasm/worldgen.cpp`), but this file defines:
- What entities exist
- What blocks are available
- How those blocks look and behave

**For modding**: Copy this file, change the scene definition, update HTML import, document your changes. The engine itself remains untouched.

---

*Document created: 2025-11-02*
*Author: Claude Code*
