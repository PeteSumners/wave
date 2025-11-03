# WAVE Engine - Terrain Mods Reference

Quick reference for the terrain modification functions in `main-duplicate.ts`.

## Available Terrain Mods

### 1. Flat Platform (ACTIVE by default)
**Function**: `createFlatPlatform()`
**Location**: Lines 1287-1323

**What it does**:
- Creates a 9×9 flat grass platform at spawn point
- Fixed height: Y=70 (adjustable)
- Fills underground with stone
- Clears floating terrain above

**Visual Result**:
```
Top view (9×9):
G G G G G G G G G
G G G G G G G G G
G G G G G G G G G
G G G G P G G G G  ← P = Player spawn (1,1)
G G G G G G G G G
G G G G G G G G G
G G G G G G G G G
G G G G G G G G G
G G G G G G G G G

Side view:
     Sky
  - - - - - -
  G G G G G G     ← Grass surface (Y=69)
  S S S S S S     ← Stone fill (Y=0-68)
```

**Parameters you can adjust**:
```typescript
const platformRadius = 4;    // Size: (radius*2+1)×(radius*2+1)
const platformHeight = 70;   // Height level (try 65, 75, 80)
```

**Use case**: Perfect for building spawn structures, guaranteed flat area

---

### 2. Hill
**Function**: `createHill()`
**Location**: Lines 1327-1373

**What it does**:
- Creates smooth circular hill
- Located 5 blocks east + 5 blocks south of spawn
- Parabolic height falloff (smooth, natural shape)
- Layered: stone → dirt → grass top

**Visual Result**:
```
Top view:
        . . . . .
      . d d d d d .
    . d d D D D d d .
    . d D D D D D d .   ← d=dirt, D=peak
    . d d D D D d d .
      . d d d d d .
        . . . . .

Side view:
      G           ← Grass top
    G D G
  G D D D G       ← Dirt layers
  S S S S S       ← Stone base
```

**Parameters you can adjust**:
```typescript
const hillOffsetX = 5;      // East/West offset from spawn
const hillOffsetZ = 5;      // North/South offset from spawn
const hillRadius = 4;       // Base radius (try 3, 5, 6)
const hillHeight = 5;       // Peak height (try 3, 7, 10)
```

**Math**: Height at distance `d` = `hillHeight × (1 - d/radius)²`
- Center (d=0): Full height
- Edge (d=radius): Height 0
- Halfway (d=radius/2): 25% height

**Use case**: Natural terrain features, vantage points, obstacles

---

### 3. Pond
**Function**: `createPond()`
**Location**: Lines 1377-1426

**What it does**:
- Digs circular depression
- Fills with water blocks
- Sand bottom
- Located 6 blocks west + 2 blocks south of spawn

**Visual Result**:
```
Top view:
      W W W
    W W W W W
    W W W W W       ← W = Water surface
    W W W W W
      W W W

Side view:
              Terrain
  - - - W W W - - -    ← Water surface
      ~ ~ ~ ~          ← Water body
      ~ ~ ~ ~
      S S S            ← Sand bottom
```

**Parameters you can adjust**:
```typescript
const pondOffsetX = -6;     // East/West offset (negative = west)
const pondOffsetZ = 2;      // North/South offset (positive = south)
const pondRadius = 3;       // Surface radius (try 2, 4, 5)
const pondDepth = 3;        // Depth at center (try 2, 4, 5)
```

**Math**: Depth at distance `d` = `pondDepth × (1 - d/radius)`
- Center (d=0): Full depth
- Edge (d=radius): Depth 0 (shallow)

**Water mechanics**:
- Water is non-solid (can swim through)
- Semi-transparent (80% opacity)
- Animated sparkle effect

**Use case**: Decorative features, swimming challenges, moats

---

### 4. Stone Path
**Function**: `createPath()`
**Location**: Lines 1430-1455

**What it does**:
- Stone path heading north from spawn
- Follows terrain height (not flat)
- Clears 2 blocks above path (prevents overhang)
- 5 blocks wide

**Visual Result**:
```
Top view:
  S S S S S
  S S S S S
  S S S S S       ← S = Stone path
  S S S S S       ↑ North direction
  S S P S S       ← P = Player spawn
  (continues 15 blocks north)

Side view (follows terrain):
      S S           ← Path on hill
    S S             ← Path on slope
  S S S             ← Path in valley
```

**Parameters you can adjust**:
```typescript
const pathLength = 15;      // How far north (try 10, 20, 30)
const pathWidth = 2;        // Half-width: total = (width*2+1)
```

**Use case**: Spawn roads, marking directions, connecting areas

---

## How to Use

### Activate a Single Mod

Edit `main-duplicate.ts` around line 1462:

```typescript
// Comment out all except one:
createFlatPlatform();   // ← Uncommented (active)
// createHill();        // ← Commented (inactive)
// createPond();        // ← Commented (inactive)
// createPath();        // ← Commented (inactive)
```

Then rebuild:
```bash
./scripts/build
```

### Combine Multiple Mods

Uncomment multiple functions:

```typescript
createFlatPlatform();   // Platform at spawn
createHill();           // Hill to the east
createPond();           // Pond to the west
createPath();           // Path heading north
```

**Result**: All 4 terrain features at once!

```
Map view:
        N
        ↑
    P P P P P   ← Path
    P P P P P
    P P P P P
W W   P S P       H H H
W W W P P       H H H H   ← H = Hill
  W W   P       H H H
      S S S
      S S S     ← S = Platform (spawn)
      S S S
```

---

## Customization Examples

### Giant Platform

```typescript
const platformRadius = 10;   // 21×21 platform
const platformHeight = 80;   // Higher up
```

### Steep Mountain

```typescript
const hillHeight = 15;       // Tall peak
const hillRadius = 6;        // Wide base
```

### Deep Lake

```typescript
const pondRadius = 6;        // Larger surface
const pondDepth = 8;         // Very deep
```

### Highway

```typescript
const pathLength = 50;       // Long road
const pathWidth = 4;         // 9 blocks wide
```

---

## Understanding the Techniques

### 1. Distance-Based Shapes (Circles)

**Used in**: Hill, Pond

```typescript
const distance = Math.sqrt(dx * dx + dz * dz);
if (distance <= radius) {
  // Inside circle
}
```

**Why**: Creates smooth, natural circular features

**Formula**: Pythagorean distance from center
- `dx` = x offset from center
- `dz` = z offset from center
- `distance` = straight-line distance

---

### 2. Height Falloff Functions

**Linear falloff** (gentle slope):
```typescript
const height = maxHeight * (1 - distance / radius);
```

**Parabolic falloff** (steeper, more natural):
```typescript
const height = maxHeight * (1 - distance / radius) ** 2;
```

**Used in Hill**:
```typescript
const heightFactor = 1 - (distance / hillRadius);
const addedHeight = int(hillHeight * heightFactor * heightFactor);
```

**Graph**:
```
Height
  ^
  |   /\          ← Parabolic (smooth peak)
  |  /  \
  | /    \
  |/      \___
  +-----------> Distance
  0          radius
```

---

### 3. Multi-Layer Terrain

**Pattern**: Stone → Dirt → Grass

```typescript
for (let y = bottom; y <= top; y++) {
  if (y === top) {
    env.setBlock(x, y, z, blocks.grass);      // Surface
  } else if (y > top - 3) {
    env.setBlock(x, y, z, blocks.dirt);       // Subsurface
  } else {
    env.setBlock(x, y, z, blocks.stone);      // Deep
  }
}
```

**Why**: Matches natural Minecraft-style terrain generation

**Visual**:
```
Y Layer:
75  G  ← Grass (surface)
74  D  ← Dirt
73  D  ← Dirt
72  D  ← Dirt
71  S  ← Stone
70  S  ← Stone (continues down)
```

---

### 4. Terrain-Following vs Fixed Height

**Terrain-following** (Path):
```typescript
const terrainHeight = env.getBaseHeight(worldX, worldZ);
env.setBlock(worldX, terrainHeight, worldZ, blocks.stone);
```
- Adapts to hills and valleys
- Natural integration

**Fixed height** (Platform):
```typescript
const platformHeight = 70;
env.setBlock(worldX, platformHeight, worldZ, blocks.grass);
```
- Ignores terrain
- Creates flat surface

---

## Advanced: Combining Techniques

### Mesa (Flat-Top Hill)

```typescript
const createMesa = () => {
  const radius = 5;
  const baseHeight = env.getBaseHeight(spawnX, spawnZ);
  const mesaHeight = baseHeight + 10;

  for (let dx = -radius; dx <= radius; dx++) {
    for (let dz = -radius; dz <= radius; dz++) {
      const distance = Math.sqrt(dx * dx + dz * dz);

      if (distance <= radius) {
        // Steep sides (no falloff)
        for (let y = baseHeight; y <= mesaHeight; y++) {
          const worldX = int(spawnX + dx);
          const worldZ = int(spawnZ + dz);

          if (y === mesaHeight) {
            env.setBlock(worldX, y, worldZ, blocks.grass);
          } else {
            env.setBlock(worldX, y, worldZ, blocks.stone);
          }
        }
      }
    }
  }
};
```

**Result**: Cylinder with flat top (mesa/butte)

---

### Crater

```typescript
const createCrater = () => {
  const radius = 6;
  const baseHeight = env.getBaseHeight(spawnX, spawnZ);

  for (let dx = -radius; dx <= radius; dx++) {
    for (let dz = -radius; dz <= radius; dz++) {
      const distance = Math.sqrt(dx * dx + dz * dz);

      if (distance <= radius) {
        // Inverse parabola (depression)
        const heightFactor = distance / radius;  // Inverted!
        const digDepth = int(5 * heightFactor * heightFactor);

        const worldX = int(spawnX + dx);
        const worldZ = int(spawnZ + dz);

        // Dig out
        for (let y = baseHeight - digDepth; y <= baseHeight; y++) {
          env.setBlock(worldX, y, worldZ, kEmptyBlock);
        }
      }
    }
  }
};
```

**Result**: Bowl-shaped depression (impact crater)

---

### Spiral Tower

```typescript
const createSpiralTower = () => {
  const height = 20;
  const baseHeight = env.getBaseHeight(spawnX, spawnZ);

  for (let y = 0; y < height; y++) {
    const angle = (y / height) * Math.PI * 4;  // 2 full rotations
    const radius = 3;

    const dx = int(Math.cos(angle) * radius);
    const dz = int(Math.sin(angle) * radius);

    const worldX = int(spawnX + dx);
    const worldY = int(baseHeight + y);
    const worldZ = int(spawnZ + dz);

    env.setBlock(worldX, worldY, worldZ, blocks.stone);
  }
};
```

**Result**: Helix/spiral staircase

---

## Performance Notes

### Block Placement Cost

Each `setBlock()` call:
1. Updates voxel data in chunk
2. Marks chunk dirty (needs remesh)
3. Updates lighting
4. Triggers mesh regeneration

**Impact**:
- Platform: 9×9×70 = ~5,600 blocks = ~100ms
- Hill: ~200 blocks = ~5ms
- Pond: ~150 blocks = ~5ms
- Path: 15×5 = 75 blocks = ~2ms

**Optimization**: Group modifications by chunk to reduce remeshing

---

## Troubleshooting

### "Terrain mod doesn't appear"

**Check**:
1. Is the function uncommented? (line 1462)
2. Did you rebuild? (`./scripts/build`)
3. Did you refresh browser?
4. Check console for errors (F12 → Console)

### "Performance is slow"

**Solutions**:
- Reduce radius/size
- Reduce height/depth
- Activate fewer mods at once
- Clear browser cache

### "Terrain looks wrong"

**Check**:
- Are coordinates correct? (spawn is 1, 1)
- Is height calculation correct?
- Use `console.log()` to debug values

---

## Summary

| Mod | Size | Location | Shape | Use Case |
|-----|------|----------|-------|----------|
| Platform | 9×9 | Spawn (0,0) | Square | Flat building area |
| Hill | Radius 4 | East+South (5,5) | Circle | Natural feature |
| Pond | Radius 3 | West (−6,2) | Circle | Water feature |
| Path | 5×15 | North (0,−15) | Rectangle | Direction marker |

**Key concepts**:
- ✓ Distance-based shapes (circles)
- ✓ Height falloff functions (smooth curves)
- ✓ Multi-layer terrain (stone/dirt/grass)
- ✓ Terrain-following vs fixed height

**Next steps**:
- Try adjusting parameters
- Combine multiple mods
- Create custom variations (mesa, crater, spiral)

---

*Last updated: 2025-11-02*
