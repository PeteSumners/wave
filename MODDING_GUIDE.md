# WAVE Engine - Safe Modding Guide

This guide explains how to safely modify the WAVE engine scene using the duplicate file approach.

## Table of Contents
- [Philosophy](#philosophy)
- [File Structure](#file-structure)
- [Example: 3x3 Cube Structure](#example-3x3-cube-structure)
- [How to Use the Mod](#how-to-use-the-mod)
- [Understanding the Code](#understanding-the-code)
- [Advanced Modding](#advanced-modding)

---

## Philosophy

### The Duplicate File Approach

**Goal**: Modify the game scene without touching engine code

**Method**:
1. ✓ Keep `main.ts` pristine (original engine)
2. ✓ Modify `main-duplicate.ts` (your mod)
3. ✓ Switch between versions by changing HTML imports
4. ✓ Document all changes in the duplicate file

**Benefits**:
- Engine updates don't break your mods
- Easy to compare original vs modified
- Can maintain multiple mod versions
- Clear separation of engine vs content

---

## File Structure

```
wave/
├── src/
│   ├── main.ts              ← ORIGINAL (don't touch!)
│   ├── main-duplicate.ts    ← YOUR MOD (edit freely)
│   ├── engine.ts            ← Engine core (don't touch)
│   └── ...
├── index.html               ← Update imports here
└── MODDING_GUIDE.md         ← This file
```

### Switching Between Versions

**To use ORIGINAL scene**:
```html
<!-- index.html -->
<script type="module" src="target/main.js"></script>
```

**To use MODDED scene**:
```html
<!-- index.html -->
<script type="module" src="target/main-duplicate.js"></script>
```

**Note**: After editing TypeScript, rebuild:
```bash
./scripts/build
```

---

## Example: 3x3 Cube Structure

We've added a 3×3×3 stone cube at the player spawn point as a demonstration.

### What It Does

When the game loads:
1. ✓ Player spawns at world position (1, 1)
2. ✓ Engine queries terrain height at that position
3. ✓ Builds a 3×3×3 cube of stone blocks 2 blocks above terrain
4. ✓ Cube is centered on spawn point (visible landmark)

### Code Location

**File**: `src/main-duplicate.ts`

**Lines**:
- **Header comment**: Lines 10-26 (documents all modifications)
- **Function definition**: Lines 1287-1324 (`buildSpawnStructure()`)
- **Function call**: Line 1327 (executes after world refresh)

---

## How to Use the Mod

### Step 1: Build TypeScript

```bash
cd wave
./scripts/build
```

**What this does**:
- Compiles `src/main-duplicate.ts` → `target/main-duplicate.js`
- JavaScript is what the browser actually runs

### Step 2: Update HTML Import

Edit `index.html` to use the modded version:

```html
<!-- Find this line (around line 20-30): -->
<script type="module" src="target/main.js"></script>

<!-- Change to: -->
<script type="module" src="target/main-duplicate.js"></script>
```

### Step 3: Open in Browser

```bash
# If using Python server:
python scripts/serve.py

# Then open browser to:
http://localhost:8000
```

**Or** just open `index.html` directly (file://)

### Step 4: Verify the Mod

You should see:
1. **Console output**:
   ```
   Building 3x3 cube at spawn (1, 1)
   Terrain height: 65, Structure start: 67
   3x3 cube structure completed!
   ```

2. **Visual**: A 3×3×3 stone cube hovering above the spawn point

---

## Understanding the Code

### The buildSpawnStructure() Function

Let's break down the code line by line:

```typescript
const buildSpawnStructure = () => {
```
**Line 1287**: Define function (called after world initialization)

---

```typescript
  const spawnX = 1;
  const spawnZ = 1;
```
**Lines 1289-1290**: Define spawn coordinates (same as player spawn)

---

```typescript
  const terrainHeight = env.getBaseHeight(spawnX, spawnZ);
```
**Line 1294**: Query world for terrain height at (x, z)

**What `getBaseHeight()` does**:
- Returns highest solid block Y-coordinate at (x, z)
- Accounts for hills, valleys, etc.
- Ensures we build on top of terrain, not inside it

**Example**: If spawn point is on a mountain at Y=80, returns 80

---

```typescript
  const startY = terrainHeight + 2;
```
**Line 1298**: Start building 2 blocks above terrain

**Why +2?**:
- Player spawns AT terrain height
- +2 creates a small gap (player can walk under)
- Prevents spawning inside the structure

---

```typescript
  console.log(`Building 3x3 cube at spawn (${spawnX}, ${spawnZ})`);
  console.log(`Terrain height: ${terrainHeight}, Structure start: ${startY}`);
```
**Lines 1300-1301**: Debug output to browser console

**How to view**: Open browser DevTools (F12) → Console tab

---

```typescript
  for (let dx = 0; dx < 3; dx++) {        // 3 blocks wide (X axis)
    for (let dy = 0; dy < 3; dy++) {      // 3 blocks tall (Y axis)
      for (let dz = 0; dz < 3; dz++) {    // 3 blocks deep (Z axis)
```
**Lines 1304-1306**: Triple nested loop creates 3×3×3 = 27 blocks

**Loop structure**:
```
dx: 0, 1, 2  (X dimension)
  dy: 0, 1, 2  (Y dimension)
    dz: 0, 1, 2  (Z dimension)
```

**Iteration order** (27 total):
1. (0,0,0), (0,0,1), (0,0,2)  ← First column
2. (0,1,0), (0,1,1), (0,1,2)  ← Second column
3. ... (continues for all 27 positions)

---

```typescript
  const worldX = int(spawnX + dx - 1);  // -1, 0, +1 (centered)
  const worldY = int(startY + dy);       // startY to startY+2
  const worldZ = int(spawnZ + dz - 1);  // -1, 0, +1 (centered)
```
**Lines 1309-1311**: Convert loop indices to world coordinates

**Centering math**:
- `dx - 1`: Converts 0,1,2 → -1,0,+1 (centered on spawn)
- `dy`: No offset (builds upward from startY)
- `dz - 1`: Same centering as X

**Example** (spawnX=1, spawnZ=1, startY=67):
| dx | dz | worldX | worldZ | Result |
|----|----| -------|--------|--------|
| 0 | 0 | 0 | 0 | Northwest corner |
| 1 | 1 | 1 | 1 | Center (spawn point) |
| 2 | 2 | 2 | 2 | Southeast corner |

**Visual (top-down view)**:
```
(0,0) (1,0) (2,0)
(0,1) (1,1) (2,1)  ← (1,1) is spawn center
(0,2) (1,2) (2,2)
```

---

```typescript
  env.setBlock(worldX, worldY, worldZ, blocks.stone);
```
**Line 1314**: Place a stone block at calculated position

**What `setBlock()` does**:
1. Updates voxel data in chunk
2. Marks chunk as "dirty" (needs remeshing)
3. Updates lighting if needed
4. Queues mesh regeneration

**Parameters**:
- `worldX, worldY, worldZ`: Integer world coordinates
- `blocks.stone`: Block type ID (from line 1273)

---

```typescript
  // Alternative: Use different blocks for variety
  // env.setBlock(worldX, worldY, worldZ, blocks.trunk);  // Wood cube
  // env.setBlock(worldX, worldY, worldZ, blocks.sand);   // Sand cube
```
**Lines 1316-1318**: Commented alternatives (easy to swap)

**To try different blocks**:
1. Comment out line 1314 (add `//` at start)
2. Uncomment one alternative (remove `//`)
3. Rebuild: `./scripts/build`
4. Refresh browser

---

```typescript
  console.log('3x3 cube structure completed!');
};
```
**Line 1323**: Completion message

**Line 1324**: End of function definition

---

```typescript
  buildSpawnStructure();
```
**Line 1327**: Call the function to actually build the structure

**Timing**: Happens AFTER `env.refresh()` completes
- World chunks are loaded
- Terrain is generated
- Height queries work correctly

---

## Advanced Modding

### Custom Structures

#### Hollow Cube (Shell Only)

```typescript
const buildHollowCube = () => {
  const spawnX = 1, spawnZ = 1;
  const terrainHeight = env.getBaseHeight(spawnX, spawnZ);
  const startY = terrainHeight + 2;

  for (let dx = 0; dx < 3; dx++) {
    for (let dy = 0; dy < 3; dy++) {
      for (let dz = 0; dz < 3; dz++) {
        // Only place blocks on the OUTER SHELL
        const isOuterShell =
          dx === 0 || dx === 2 ||  // X edges
          dy === 0 || dy === 2 ||  // Y edges
          dz === 0 || dz === 2;    // Z edges

        if (!isOuterShell) continue;  // Skip interior

        const worldX = int(spawnX + dx - 1);
        const worldY = int(startY + dy);
        const worldZ = int(spawnZ + dz - 1);

        env.setBlock(worldX, worldY, worldZ, blocks.stone);
      }
    }
  }
};
```

**Result**: 26 blocks (27 - 1 hollow center)

---

#### Tower Structure

```typescript
const buildTower = () => {
  const spawnX = 1, spawnZ = 1;
  const terrainHeight = env.getBaseHeight(spawnX, spawnZ);
  const startY = terrainHeight;

  // Build a 3x3 base, 10 blocks tall
  for (let dx = 0; dx < 3; dx++) {
    for (let dz = 0; dz < 3; dz++) {
      const worldX = int(spawnX + dx - 1);
      const worldZ = int(spawnZ + dz - 1);

      // 10 blocks tall
      for (let height = 0; height < 10; height++) {
        const worldY = int(startY + height);

        // Alternate stone and wood for stripes
        const block = (height % 2 === 0) ? blocks.stone : blocks.trunk;
        env.setBlock(worldX, worldY, worldZ, block);
      }
    }
  }
};
```

**Result**: 90 blocks (3×3×10) with alternating stone/wood layers

---

#### Pyramid

```typescript
const buildPyramid = () => {
  const spawnX = 1, spawnZ = 1;
  const terrainHeight = env.getBaseHeight(spawnX, spawnZ);
  const startY = terrainHeight + 1;

  // 5-layer pyramid (5x5 base, 1x1 top)
  for (let layer = 0; layer < 5; layer++) {
    const size = 5 - layer;  // 5, 4, 3, 2, 1
    const offset = Math.floor(layer / 2);  // Centering offset

    for (let dx = 0; dx < size; dx++) {
      for (let dz = 0; dz < size; dz++) {
        const worldX = int(spawnX + dx - Math.floor(size / 2));
        const worldY = int(startY + layer);
        const worldZ = int(spawnZ + dz - Math.floor(size / 2));

        env.setBlock(worldX, worldY, worldZ, blocks.sand);
      }
    }
  }
};
```

**Result**: 55 blocks (5² + 4² + 3² + 2² + 1²) forming pyramid

---

### Multi-Block Types

#### Checkered Pattern

```typescript
const buildCheckeredCube = () => {
  const spawnX = 1, spawnZ = 1;
  const terrainHeight = env.getBaseHeight(spawnX, spawnZ);
  const startY = terrainHeight + 2;

  for (let dx = 0; dx < 4; dx++) {
    for (let dy = 0; dy < 4; dy++) {
      for (let dz = 0; dz < 4; dz++) {
        const worldX = int(spawnX + dx - 2);
        const worldY = int(startY + dy);
        const worldZ = int(spawnZ + dz - 2);

        // Checkered pattern: even sum = stone, odd sum = snow
        const sum = dx + dy + dz;
        const block = (sum % 2 === 0) ? blocks.stone : blocks.snow;

        env.setBlock(worldX, worldY, worldZ, block);
      }
    }
  }
};
```

**Result**: 64-block 4×4×4 cube with checkered stone/snow pattern

---

### Conditional Structures

#### Only Build if Flat Terrain

```typescript
const buildIfFlat = () => {
  const spawnX = 1, spawnZ = 1;
  const terrainHeight = env.getBaseHeight(spawnX, spawnZ);

  // Check 3x3 area around spawn
  let isFlat = true;
  for (let dx = -1; dx <= 1; dx++) {
    for (let dz = -1; dz <= 1; dz++) {
      const height = env.getBaseHeight(spawnX + dx, spawnZ + dz);
      if (Math.abs(height - terrainHeight) > 1) {
        isFlat = false;
        break;
      }
    }
    if (!isFlat) break;
  }

  if (isFlat) {
    console.log('Terrain is flat, building structure...');
    // Build your structure here
  } else {
    console.log('Terrain too uneven, skipping structure');
  }
};
```

---

## Debugging Your Mods

### Console Logging

**View logs**: Browser DevTools (F12) → Console

**Example**:
```typescript
console.log('Building structure...');
console.log(`Position: (${x}, ${y}, ${z})`);
console.log(`Block type: ${blocks.stone}`);
```

### Visual Debugging

**Place marker blocks**:
```typescript
// Red wool at corners for debugging (if you add red wool block)
env.setBlock(minX, minY, minZ, blocks.redWool);  // Southwest bottom
env.setBlock(maxX, maxY, maxZ, blocks.redWool);  // Northeast top
```

### Testing Changes

**Quick test cycle**:
1. Edit `main-duplicate.ts`
2. Run `./scripts/build`
3. Refresh browser (Ctrl+R or Cmd+R)
4. Check console for errors

**Common errors**:
- `TypeError: Cannot read property 'stone' of undefined`
  - **Fix**: Make sure you're accessing `blocks` after it's defined
- `Block not placing`
  - **Fix**: Check coordinates are within loaded chunks
- `Structure appears underground`
  - **Fix**: Use `getBaseHeight()` + offset

---

## Best Practices

### 1. Document Your Changes

Always add a header comment:
```typescript
//////////////////////////////////////////////////////////////////////////////
// SCENE MODIFICATIONS
// Date: YYYY-MM-DD
// Author: Your Name
//
// Changes:
// - Added X feature at line YYY
// - Modified Z behavior at line AAA
//
// Purpose:
// - Testing new gameplay mechanic
// - Creating custom spawn area
//////////////////////////////////////////////////////////////////////////////
```

### 2. Use Descriptive Function Names

```typescript
// Good
const buildSpawnTower = () => { ... };
const createWelcomeSign = () => { ... };
const setupDebugMarkers = () => { ... };

// Bad
const doStuff = () => { ... };
const build = () => { ... };
const x = () => { ... };
```

### 3. Add Comments for Complex Logic

```typescript
// Calculate pyramid layer size
// Layer 0: 5x5 (base)
// Layer 1: 4x4
// Layer 2: 3x3
// Layer 3: 2x2
// Layer 4: 1x1 (top)
const size = 5 - layer;
```

### 4. Test Incrementally

Don't write 200 lines then test. Instead:
1. Write 10-20 lines
2. Build and test
3. Verify it works
4. Continue

### 5. Keep Backups

```bash
# Before major changes:
cp main-duplicate.ts main-duplicate.ts.backup

# If things break:
cp main-duplicate.ts.backup main-duplicate.ts
```

---

## Troubleshooting

### "Structure doesn't appear"

**Check**:
1. Did you call the function? (`buildSpawnStructure()`)
2. Is it called AFTER `env.refresh()`?
3. Are coordinates within loaded chunks? (±15 blocks from spawn)
4. Check browser console for errors

### "Blocks appear in wrong location"

**Check**:
1. Coordinate math (centering offsets)
2. Use `console.log()` to print positions
3. Place single test block first

### "Game crashes on load"

**Check**:
1. TypeScript compilation errors: `./scripts/build`
2. Browser console errors: F12 → Console
3. Did you use `int()` for coordinates?
4. Are block types valid? (e.g., `blocks.stone` not `'stone'`)

---

## Summary

### Safe Modding Workflow

1. ✓ Edit `main-duplicate.ts` (never `main.ts`)
2. ✓ Add header comment documenting changes
3. ✓ Write modular functions (don't dump code in `main()`)
4. ✓ Call functions after `env.refresh()`
5. ✓ Build: `./scripts/build`
6. ✓ Update `index.html` import
7. ✓ Test in browser
8. ✓ Check console for errors/logs

### Key Functions

| Function | Purpose | Example |
|----------|---------|---------|
| `env.setBlock(x, y, z, block)` | Place block | `env.setBlock(1, 65, 1, blocks.stone)` |
| `env.getBaseHeight(x, z)` | Query terrain height | `const h = env.getBaseHeight(1, 1)` |
| `env.getBlock(x, y, z)` | Query block type | `const b = env.getBlock(1, 65, 1)` |

### Coordinate System

- **X**: East (+) / West (-)
- **Y**: Up (+) / Down (-) - Height from 0 to 255
- **Z**: South (+) / North (-)
- **Origin**: (0, 0, 0) is northwest corner at bedrock level

---

**Happy modding! 🎮**

*Last updated: 2025-11-02*
