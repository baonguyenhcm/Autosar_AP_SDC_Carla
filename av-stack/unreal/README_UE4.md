# av-stack → Unreal Engine 4 (3D replay)

Replays the `av_trace.csv` produced by `pipeline_demo` inside UE4. An
`AAvReplayActor` loads the trace and drives the ego (and its EKF-estimate ghost)
along the recorded path each frame. Out of the box it spawns coloured boxes, a
ground plane and a chase camera, so you see the car drive and stop for the
obstacle with zero asset setup. You can later assign your own vehicle meshes.

Tested against **UE 4.27** (the APIs used also exist in 4.26 and UE5).

---

## 1. You need a C++ project

If your project is Blueprint-only, convert it: in the editor
**Tools → New C++ Class → None → Create Class**. This generates the
`Source/` folder and a Visual Studio solution.

## 2. Add the two source files

Copy into your game module's source folder, e.g.
`Source/<YourProject>/`:

```
AvReplayActor.h
AvReplayActor.cpp
```

The class has **no `PROJECT_API` macro**, so it must live in your primary game
module (the one named after your project). If you get unresolved-symbol linker
errors because you put it in a different module, add `YOURMODULE_API` before the
class name in the header.

## 3. Regenerate + build

- Right-click the `.uproject` → **Generate Visual Studio project files**.
- Open the `.sln`, build **Development Editor**, or just reopen the project and
  click **Compile** in the editor toolbar.

## 4. Put the trace where the actor can find it

By default the actor reads `av_trace.csv` from the **project directory**
(the folder that holds your `.uproject`). Copy it there:

```
<YourProject>/av_trace.csv
```

Or set an absolute path in the actor's **CSV File Name** property (step 5).
A ready-made trace lives at `av-stack/viz/sample_av_trace.csv`.

## 5. Drop the actor into a level

- In the **Place Actors** panel search for **AvReplayActor** and drag it into
  the level at the origin (0,0,0 is easiest).
- Press **Play**. The teal box is the true ego, the blue box is the EKF
  estimate, the red box is the obstacle, and the chase camera follows along.

### Useful properties (Details panel)

| Property | Default | Meaning |
|---|---|---|
| `CSV File Name` | `av_trace.csv` | File under the project dir, or an absolute path |
| `Time Scale` | `1.0` | Playback speed multiplier |
| `Loop` | `true` | Restart at the end |
| `Show Ghost` | `true` | Show the EKF-estimate car |
| `Spawn Chase Camera` | `true` | Auto-follow camera |
| `Show Telemetry` | `true` | On-screen speed / throttle / brake |
| `Ego / Ghost / Obstacle Actor Override` | *none* | Assign your own actors to drive instead of the boxes |

---

## Swapping in a real vehicle mesh

1. Add your vehicle actor (a Static/Skeletal Mesh actor, or the UE4 vehicle
   template blueprint) to the level.
2. On the AvReplayActor, set **Ego Actor Override** to that actor. Its transform
   is now driven by the trace and the teal box is hidden.
3. Mesh pivots vary: if the car floats or sinks, adjust the mesh's pivot, or
   change the `CarZ` constant (currently `75.0` uu = 0.75 m) in
   `LoadCsv()` so the wheels rest on `z = 0`.

Do the same with **Obstacle Actor Override** for a real obstacle prop.

---

## Coordinate & unit conventions (important)

av-stack uses the robotics convention; UE4 uses a left-handed game convention.
The actor converts on load:

| Quantity | av-stack | UE4 | Conversion |
|---|---|---|---|
| Length | metres | centimetres (uu) | `× 100` (`MetersToUU`) |
| X (forward) | `x` | `X` | `X = x·100` |
| Y (lateral) | `y` **left**+ | `Y` **right**+ | `Y = −y·100` |
| Heading | `yaw` CCW, rad | `Yaw` CW, deg | `Yaw = −yaw·180/π` |
| Up | `z` | `Z` | same |

If you extend the trace with a curved path and it appears mirrored, the Y and
yaw sign flips above are the first thing to check.

---

## Live streaming (UDP) — watch the pipeline in real time

CSV replay is deterministic and needs no networking — the best default for
repeatable demos. For a real-time "watch it drive as it computes" view, stream
the pipeline over UDP into `AAvUdpReceiverActor` instead. Both ends are wired up.

### Sender — build the demo with UDP enabled

```bash
cd git_repos/av-stack
cmake -S tools -B build_udp -DAV_UDP=ON
cmake --build build_udp
./build_udp/pipeline_demo      # now paced at ~real time, streaming to 127.0.0.1:9999
```

`-DAV_UDP=ON` compiles in `tools/udp_sender.hpp` (POSIX + Winsock), sends the
`# lane` / `# obstacle` scenario lines once, then one datagram per 0.1 s step,
and paces the loop to real time. The CSV is still written, so nothing is lost.

### Receiver — add the actor in UE4

1. Add `AvUdpReceiverActor.h/.cpp` to your game module (alongside the replay
   actor).
2. Enable the socket modules in `Source/<YourProject>/<YourProject>.Build.cs`:

   ```csharp
   PublicDependencyModuleNames.AddRange(new[] { "Sockets", "Networking" });
   ```

3. Regenerate project files, rebuild.
4. Place an **AvUdpReceiverActor** in the level (its `Port` defaults to 9999),
   press **Play**, then start the streaming demo. The ego moves live.

### Notes

- **Run order doesn't matter** — UDP is connectionless. The receiver binds the
  port at Play; late-starting or restarting the demo just resumes the stream.
- **Same machine**: if UE runs on Windows and the demo in WSL2, `127.0.0.1`
  works because WSL2 forwards localhost. Across machines, change the target IP
  in `pipeline_demo.cpp` (`UdpSender udp("<UE-host-ip>", 9999)`) and allow the
  port through the firewall.
- `SmoothingSpeed` on the actor controls how tightly the car tracks incoming
  packets (higher = snappier, lower = smoother).
- Keep the CSV/replay path around — it's still the right tool for the 2D viewer
  and for regression demos.
