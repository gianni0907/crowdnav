# Enhanced Crowd Simulation Plugin

This plugin provides realistic crowd behavior with different actor types for Gazebo 11.

## Behavior Types

### 1. **STATIC** - Standing Still
Actors remain at their initial position.

```xml
<plugin name="actor_plugin" filename="libEnhancedActorPlugin.so">
  <behavior>static</behavior>
</plugin>
```

### 2. **WALKING_SOLO** - Random Waypoint Walking
Actors walk between randomly generated waypoints.

```xml
<plugin name="actor_plugin" filename="libEnhancedActorPlugin.so">
  <behavior>walking_solo</behavior>
  <velocity>1.0</velocity>
  <min_x>-10.0</min_x>
  <max_x>10.0</max_x>
  <min_y>-10.0</min_y>
  <max_y>10.0</max_y>
  <num_waypoints>5</num_waypoints>
</plugin>
```

### 3. **WALKING_PAIR** - Walking Together
Two actors walk side-by-side following the same waypoints.

```xml
<!-- Person A -->
<plugin name="actor_plugin" filename="libEnhancedActorPlugin.so">
  <behavior>walking_pair</behavior>
  <partner>person_b_name</partner>
  <formation_distance>0.8</formation_distance>  <!-- Distance between pair -->
  <velocity>0.9</velocity>
  <min_x>-5.0</min_x>
  <max_x>5.0</max_x>
  <min_y>-5.0</min_y>
  <max_y>5.0</max_y>
  <num_waypoints>4</num_waypoints>
</plugin>

<!-- Person B - same waypoints, different starting pose -->
<plugin name="actor_plugin" filename="libEnhancedActorPlugin.so">
  <behavior>walking_pair</behavior>
  <partner>person_a_name</partner>
  <formation_distance>0.8</formation_distance>
  <!-- Same waypoint bounds as partner -->
  <velocity>0.9</velocity>
  <min_x>-5.0</min_x>
  <max_x>5.0</max_x>
  <min_y>-5.0</min_y>
  <max_y>5.0</max_y>
  <num_waypoints>4</num_waypoints>
</plugin>
```

**Important for pairs:**
- Both actors must have the SAME waypoint bounds (min_x, max_x, min_y, max_y, num_waypoints)
- Both must specify each other as partners
- Start them close together (formation_distance apart)

### 4. **STATIC_PAIR** - Standing Together
Two actors remain stationary but keep a shoulder-to-shoulder formation.

```xml
<plugin name="actor_plugin" filename="libEnhancedActorPlugin.so">
  <behavior>static_pair</behavior>
  <partner>friend_name</partner>
  <formation_distance>0.8</formation_distance>
</plugin>
```

The follower automatically mirrors the leader's pose and offset, so only one member of the pair needs to be positioned manually.

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `behavior` | string | walking_solo | Behavior type: static, static_pair, walking_solo, walking_pair |
| `velocity` | double | 0.8 | Walking speed in m/s |
| `min_x`, `max_x` | double | -5.0, 5.0 | X bounds for waypoint generation |
| `min_y`, `max_y` | double | -5.0, 5.0 | Y bounds for waypoint generation |
| `num_waypoints` | int | 5 | Number of random waypoints |
| `partner` | string | - | Name of partner actor (for walking_pair) |
| `formation_distance` | double | 0.8 | Distance between pair members |
| `seed` | uint | hash(actor name) | Optional RNG seed for deterministic waypoint generation |
| `min_waypoint_separation` | double | half of room diagonal | Minimum distance enforced between consecutive waypoints |
| `scenario_seed` | uint | 0 | Optional per-actor modifier combined with the global seed |

Set the environment variable `CROWD_SIM_SEED=<value>` before launching Gazebo to change the entire scenario with a single number. Every actor automatically XORs this global seed with its internal identifier, so you only need to tweak one value to generate a new deterministic crowd layout.

## Building

```bash
# Copy the plugin to your package
cp EnhancedActorPlugin.cc ~/crowdnav_ws/src/crowdnav/labrob_gazebo_worlds/

# Update CMakeLists.txt to build it
add_library(EnhancedActorPlugin SHARED EnhancedActorPlugin.cc)
target_link_libraries(EnhancedActorPlugin ${GAZEBO_LIBRARIES})

# Build
cd ~/crowdnav_ws/build/labrob_gazebo_worlds
cmake ~/crowdnav_ws/src/crowdnav/labrob_gazebo_worlds
make
```

## Example: 10 Actor Crowd

See `crowd_world_example.world` for a complete example with:
- 3 static actors
- 2 pairs walking together (4 actors)
- 3 solo walkers

For a structured indoor scene, check `worlds/labrob_2rooms_10humans_plugin.world`, which keeps:
- 2 static pairs (4 actors total) split between the large and small rooms
- 2 walking pairs (another 4 actors) covering both rooms
- 2 walking solos handling the remaining traffic
All actors clamp their random waypoint generation to the room they spawn in, and each mover sets `num_waypoints` to 2 for simple back-and-forth motion.

## Tips for Realistic Crowds

1. **Vary velocities**: People walk at different speeds (0.7 - 1.2 m/s)
2. **Different waypoint areas**: Give different groups different zones
3. **Mix behaviors**: Combine static, solo, and pairs
4. **Stagger start positions**: Don't place everyone at origin
5. **Formation distance**: 0.6-1.0m feels natural for pairs

## Future Enhancements

Ideas you could add:
- **Groups of 3-4**: Extend walking_pair to walking_group
- **Following behavior**: One actor follows another
- **Obstacle avoidance**: Actors avoid each other (see official ActorPlugin for reference)
- **Speed variation**: Random speed changes to simulate hesitation
- **Stopping behavior**: Actors pause randomly at waypoints
- **Different animations**: Standing, sitting, waving

## Troubleshooting

**Pairs not staying together:**
- Check that both have IDENTICAL waypoint parameters
- Ensure they start within formation_distance of each other
- Check that partner names are correct

**Actors teleporting:**
- Make sure you don't have `<script>` tags in your actor definitions
- Only one plugin per actor

**Actors sinking/floating:**
- The Z height is set to 1.2138 (standard for walk.dae model)
- If using different models, adjust this in the code