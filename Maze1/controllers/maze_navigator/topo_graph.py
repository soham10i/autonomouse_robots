"""Topological (graph) layer over the metric occupancy grid.

This is the "memory table" — a hybrid/topometric map. The metric layer
(``Grid2D``) answers *what is solid*; this graph answers *where the
decision points are and which branches are still unexplored*.

Step-2 scope (this file): the graph is built PASSIVELY. ``observe()`` is
called every controller tick; it detects junctions / dead-ends from the
lidar scan, creates or merges nodes, and links edges as the robot drives
between them. It does NOT influence motion yet — that is step 3.

Data structures
---------------
``Opening``   one corridor mouth at a node: world bearing, angular width,
              exploration status, and (once driven) the edge it became.
``TopoNode``  a junction / dead-end / the start pose. Holds its metric
              XY and its list of openings.
``TopoEdge``  a corridor actually traversed between two nodes: the two
              endpoint node ids, the driven length, and whether the
              straight line between the endpoints crosses poison.
``TopoGraph`` nodes + edges + an adjacency list (== the memory table in
              the hand-drawn design). Exposes ``dijkstra`` and
              ``nearest_unexplored`` so step 3 can route on it directly.

Node taxonomy (from the 360° lidar gap count)
---------------------------------------------
  openings >= 3  → JUNCTION   (T, +, or more)
  openings == 1  → DEAD_END
  openings == 2  → corridor   (transit — NOT a node)
  the very first observation is always the START node.
"""
import json
import math
import os

import numpy as np

import config as C


# ---------------------------------------------------------------------------
# Tunables
# ---------------------------------------------------------------------------
# A lidar beam counts as "open" (a corridor leads this way) if it sees
# farther than this, or returns nothing at all.
OPEN_RANGE_M = 0.55
# A run of open beams is a real corridor mouth only if its angular span is
# at least this wide — filters out single-beam noise / doorway slivers.
MIN_OPENING_RAD = math.radians(18.0)
# Two node candidates within this metric distance AND with a similar
# opening count are treated as the SAME node (topological loop closure).
MERGE_RADIUS_M = 0.45
# Once a node is registered, the robot must move at least this far from it
# before a new node event can fire. Kept LARGER than MERGE_RADIUS_M so the
# robot is fully clear of a node's merge zone before the next event — this
# removes the "ping-pong" band (NODE_CLEAR < dist < MERGE_RADIUS) that
# previously re-merged the same node every tick.
NODE_CLEAR_M = 0.55
# Heading-change gate. A node is a DECISION point — the robot only makes a
# decision where it TURNS (junction branch) or turns around (dead-end).
# Cruising straight down a corridor must create NO nodes, even if the
# opening detector mis-reads the narrow corridor as a dead-end. So a node
# event is only allowed once the robot's heading has changed by at least
# this much since the previous node. This is the primary fix for the
# "fake DEAD_END every 0.45 m" spam.
NODE_MIN_TURN_RAD = math.radians(35.0)
# Robot footprint diameter (a corridor narrower than this is not passable).
ROBOT_DIAMETER_M = 2.0 * C.ROBOT_RADIUS


# ---------------------------------------------------------------------------
class Opening:
    """One corridor mouth observed at a node."""
    UNEXPLORED = "UNEXPLORED"
    EXPLORED = "EXPLORED"

    __slots__ = ("bearing", "width", "status", "edge_id")

    def __init__(self, bearing, width):
        self.bearing = bearing      # world-frame angle of the corridor (rad)
        self.width = width          # angular span of the opening (rad)
        self.status = Opening.UNEXPLORED
        self.edge_id = None         # set once this opening becomes an edge

    def as_dict(self):
        return {"bearing": round(self.bearing, 4),
                "width": round(self.width, 4),
                "status": self.status,
                "edge_id": self.edge_id}


class TopoNode:
    """A junction / dead-end / start pose."""
    JUNCTION = "JUNCTION"
    DEAD_END = "DEAD_END"
    START = "START"

    __slots__ = ("id", "xy", "kind", "openings", "visits", "t_created")

    def __init__(self, node_id, xy, kind, openings, t_created):
        self.id = node_id
        self.xy = xy                # (x, y) metres, odometry frame
        self.kind = kind
        self.openings = openings    # list[Opening]
        self.visits = 1
        self.t_created = t_created

    def n_unexplored(self):
        return sum(1 for o in self.openings
                   if o.status == Opening.UNEXPLORED)

    def as_dict(self):
        return {"id": self.id,
                "xy": [round(self.xy[0], 3), round(self.xy[1], 3)],
                "kind": self.kind,
                "visits": self.visits,
                "t_created": round(self.t_created, 2),
                "openings": [o.as_dict() for o in self.openings]}


class TopoEdge:
    """A corridor the robot actually drove between two nodes."""
    __slots__ = ("id", "a", "b", "length", "crosses_poison")

    def __init__(self, edge_id, a, b, length, crosses_poison):
        self.id = edge_id
        self.a = a                  # node id
        self.b = b                  # node id
        self.length = length        # driven distance (m), NOT Euclidean
        self.crosses_poison = crosses_poison

    def as_dict(self):
        return {"id": self.id, "a": self.a, "b": self.b,
                "length": round(self.length, 3),
                "crosses_poison": self.crosses_poison}


# ---------------------------------------------------------------------------
class TopoGraph:
    """The memory table: nodes + edges + adjacency list."""

    def __init__(self, log_fn=print):
        self.nodes = {}             # id -> TopoNode
        self.edges = {}             # id -> TopoEdge
        self.adjacency = {}         # id -> list[(neighbor_id, edge_id, length)]
        self._next_node = 0
        self._next_edge = 0
        self._log = log_fn

        # Live tracking state for observe().
        self._last_node_id = None   # node we most recently departed / are at
        self._at_node = False       # currently parked inside a node region
        self._traveled = 0.0        # odometry distance since last node
        self._prev_pose = None      # for the distance integral
        self._started = False
        self._grid = None           # latest Grid2D ref (for poison checks)
        self._heading_last_node = 0.0  # robot heading when the last node fired

    # ----------------------------- public API ---------------------------

    def observe(self, pose, ranges, angles, grid, sim_time):
        """Feed one tick of sensing. Builds the graph as a side effect.

        ``pose``   (x, y, theta) in the odometry frame.
        ``ranges`` / ``angles``  the full lidar scan (robot frame angles).
        ``grid``   the live ``Grid2D`` (used only for the poison check on
                   new edges).
        """
        x, y, th = pose
        self._grid = grid           # captured for the edge poison check

        # Integrate driven distance for edge-length bookkeeping.
        if self._prev_pose is not None:
            self._traveled += math.hypot(x - self._prev_pose[0],
                                         y - self._prev_pose[1])
        self._prev_pose = (x, y)

        # The very first observation anchors the START node.
        if not self._started:
            self._started = True
            openings = self._detect_openings(ranges, angles, th)
            nid = self._new_node((x, y), TopoNode.START, openings, sim_time)
            self._last_node_id = nid
            self._at_node = True
            self._heading_last_node = th   # seed the heading-change gate
            self._log(f"[topo] START node #{nid} @ "
                      f"({x:+.2f}, {y:+.2f})  openings={len(openings)}")
            return

        openings = self._detect_openings(ranges, angles, th)
        n_open = len(openings)

        # ── Are we still parked at the last node? clear it once we've moved.
        if self._at_node and self._last_node_id is not None:
            nx, ny = self.nodes[self._last_node_id].xy
            if math.hypot(x - nx, y - ny) > NODE_CLEAR_M:
                self._at_node = False
                self._traveled = 0.0   # start measuring this corridor

        # ── A junction or dead-end while NOT parked ⇒ node event.
        if not self._at_node:
            kind = None
            if n_open >= 3:
                kind = TopoNode.JUNCTION
            elif n_open == 1:
                kind = TopoNode.DEAD_END
            if kind is not None:
                # Heading-change gate: a node is a decision point — only
                # register one once the robot has actually turned since
                # the previous node. Cruising straight down a corridor
                # (heading ≈ constant) creates NO nodes, which kills the
                # "fake DEAD_END every 0.45 m" spam even when the opening
                # detector mis-reads a narrow corridor.
                turned = abs(_wrap(th - self._heading_last_node))
                if turned >= NODE_MIN_TURN_RAD:
                    self._node_event((x, y), kind, openings, sim_time)
                    self._heading_last_node = th

    def save_json(self, path):
        """Write the memory table (nodes + edges + adjacency) to JSON."""
        data = {
            "nodes": [self.nodes[i].as_dict() for i in sorted(self.nodes)],
            "edges": [self.edges[i].as_dict() for i in sorted(self.edges)],
            "adjacency": {str(k): v for k, v in self.adjacency.items()},
        }
        try:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            with open(path, "w") as f:
                json.dump(data, f, indent=2)
            self._log(f"[topo] wrote {path}")
        except Exception as e:
            self._log(f"[topo] save_json failed: {e}")

    def save_png(self, path, grid=None):
        """Render the graph (nodes + edges + unexplored stubs)."""
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            return
        fig, ax = plt.subplots(1, 1, figsize=(9, 9))

        # Faint occupancy backdrop, if a grid was supplied.
        if grid is not None:
            try:
                occ = grid.occupied_mask()
                ys, xs = np.where(occ.T)
                wx = grid.origin[0] + (xs + 0.5) * grid.res
                wy = grid.origin[1] + (ys + 0.5) * grid.res
                ax.scatter(wx, wy, s=1, c="0.80", marker="s")
            except Exception:
                pass

        # Edges.
        for e in self.edges.values():
            ax0, ay0 = self.nodes[e.a].xy
            bx0, by0 = self.nodes[e.b].xy
            col = "tomato" if e.crosses_poison else "0.45"
            ax.plot([ax0, bx0], [ay0, by0], "-", color=col, lw=1.8, zorder=4)
            mx, my = 0.5 * (ax0 + bx0), 0.5 * (ay0 + by0)
            ax.annotate(f"{e.length:.2f}m", (mx, my), fontsize=7,
                        color=col, zorder=5)

        # Nodes + unexplored opening stubs.
        kc = {TopoNode.JUNCTION: "royalblue",
              TopoNode.DEAD_END: "crimson",
              TopoNode.START: "forestgreen"}
        for n in self.nodes.values():
            nx, ny = n.xy
            ax.plot(nx, ny, "o", ms=13, color=kc.get(n.kind, "gray"),
                    zorder=8, mec="black")
            ax.annotate(str(n.id), (nx, ny), fontsize=9, color="white",
                        ha="center", va="center", zorder=9, fontweight="bold")
            for o in n.openings:
                if o.status == Opening.UNEXPLORED:
                    ex = nx + 0.30 * math.cos(o.bearing)
                    ey = ny + 0.30 * math.sin(o.bearing)
                    ax.plot([nx, ex], [ny, ey], "-", color="orange",
                            lw=1.4, zorder=6)

        ax.set_aspect("equal")
        ax.grid(True, alpha=0.3, ls=":")
        ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
        ax.set_title("Topological map — blue=junction, red=dead-end, "
                     "green=start\norange stubs = unexplored openings, "
                     "red edges cross poison")
        try:
            os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
            fig.tight_layout()
            fig.savefig(path, dpi=140)
            plt.close(fig)
            self._log(f"[topo] wrote {path}")
        except Exception as e:
            self._log(f"[topo] save_png failed: {e}")
            plt.close(fig)

    def summary(self):
        n_j = sum(1 for n in self.nodes.values()
                  if n.kind == TopoNode.JUNCTION)
        n_d = sum(1 for n in self.nodes.values()
                  if n.kind == TopoNode.DEAD_END)
        n_unexp = sum(n.n_unexplored() for n in self.nodes.values())
        return (f"{len(self.nodes)} nodes ({n_j} junctions, {n_d} dead-ends), "
                f"{len(self.edges)} edges, {n_unexp} unexplored openings")

    # ------------------------- graph algorithms --------------------------
    # Not used in step 2 (passive). Kept ready + unit-testable for step 3.

    def dijkstra(self, src, dst):
        """Shortest path src→dst over edges. Returns [node ids] or None."""
        if src not in self.nodes or dst not in self.nodes:
            return None
        import heapq
        dist = {src: 0.0}
        prev = {}
        pq = [(0.0, src)]
        while pq:
            d, u = heapq.heappop(pq)
            if u == dst:
                path = [u]
                while u in prev:
                    u = prev[u]
                    path.append(u)
                return path[::-1]
            if d > dist.get(u, math.inf):
                continue
            for (v, _eid, length) in self.adjacency.get(u, []):
                nd = d + length
                if nd < dist.get(v, math.inf):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        return None

    def nearest_unexplored(self, src):
        """Closest node (by graph distance) that still has an UNEXPLORED
        opening. Returns ``(node_id, [route])`` or ``(None, None)``."""
        if src not in self.nodes:
            return None, None
        import heapq
        dist = {src: 0.0}
        prev = {}
        pq = [(0.0, src)]
        best = None
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist.get(u, math.inf):
                continue
            if self.nodes[u].n_unexplored() > 0:
                best = u
                break
            for (v, _eid, length) in self.adjacency.get(u, []):
                nd = d + length
                if nd < dist.get(v, math.inf):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(pq, (nd, v))
        if best is None:
            return None, None
        route = [best]
        u = best
        while u in prev:
            u = prev[u]
            route.append(u)
        return best, route[::-1]

    # ----------------------------- internals -----------------------------

    def _detect_openings(self, ranges, angles, robot_theta):
        """Return a list of ``Opening`` from one lidar scan.

        A beam is "open" if it sees past OPEN_RANGE_M (or returns nothing).
        Consecutive open beams form a run; a run wider than MIN_OPENING_RAD
        is a corridor mouth.
        """
        if ranges is None or angles is None:
            return []
        ra = np.asarray(ranges, dtype=np.float64)
        an = np.asarray(angles, dtype=np.float64)
        n = ra.size
        if n < 8 or an.size != n:
            return []
        finite = np.isfinite(ra)
        is_open = (~finite) | (ra > OPEN_RANGE_M)
        step = abs(float(an[1] - an[0])) if n > 1 else 0.0
        if step <= 0:
            return []

        openings = []
        for (start_idx, count) in _circular_runs(is_open):
            width = count * step
            if width < MIN_OPENING_RAD:
                continue
            center_idx = (start_idx + count // 2) % n
            bearing = _wrap(robot_theta + float(an[center_idx]))
            openings.append(Opening(bearing, width))
        return openings

    def _node_event(self, xy, kind, openings, sim_time):
        """A junction / dead-end was reached while not parked at a node."""
        # Data association: merge with a nearby existing node if one fits.
        merged_id = self._find_mergeable(xy, openings)
        if merged_id is not None:
            node = self.nodes[merged_id]
            node.visits += 1
            self._log(f"[topo] node #{merged_id} REVISITED @ "
                      f"({xy[0]:+.2f}, {xy[1]:+.2f})  visits={node.visits}")
            nid = merged_id
        else:
            nid = self._new_node(xy, kind, openings, sim_time)
            self._log(f"[topo] node #{nid} created ({kind}) @ "
                      f"({xy[0]:+.2f}, {xy[1]:+.2f})  "
                      f"openings={len(openings)}")

        # Link an edge from the node we came from.
        if self._last_node_id is not None and self._last_node_id != nid:
            self._link_edge(self._last_node_id, nid, self._traveled)

        self._last_node_id = nid
        self._at_node = True
        self._traveled = 0.0

    def _find_mergeable(self, xy, openings):
        """Return id of an existing node that this observation should merge
        into (topological loop closure), or None."""
        best, best_d = None, MERGE_RADIUS_M
        for n in self.nodes.values():
            d = math.hypot(xy[0] - n.xy[0], xy[1] - n.xy[1])
            if d < best_d:
                # opening count within ±1 → compatible signature
                if abs(len(n.openings) - len(openings)) <= 1:
                    best, best_d = n.id, d
        return best

    def _new_node(self, xy, kind, openings, sim_time):
        nid = self._next_node
        self._next_node += 1
        self.nodes[nid] = TopoNode(nid, xy, kind, openings, sim_time)
        self.adjacency[nid] = []
        return nid

    def _link_edge(self, a, b, length):
        """Create an a↔b edge unless one already exists."""
        for (nb, _eid, _ln) in self.adjacency.get(a, []):
            if nb == b:
                return  # already linked — don't duplicate
        crosses = self._line_crosses_poison(a, b)
        eid = self._next_edge
        self._next_edge += 1
        self.edges[eid] = TopoEdge(eid, a, b, length, crosses)
        self.adjacency[a].append((b, eid, length))
        self.adjacency[b].append((a, eid, length))
        # Mark the opening on each node that points toward the other as
        # EXPLORED (it has now become a known edge).
        self._mark_opening_explored(a, b, eid)
        self._mark_opening_explored(b, a, eid)
        self._log(f"[topo] edge #{eid}: node {a} <-> node {b}  "
                  f"length={length:.2f}m  poison={crosses}")

    def _mark_opening_explored(self, node_id, toward_id, edge_id):
        """Flip the opening of ``node_id`` that best points at ``toward_id``
        to EXPLORED and attach the edge."""
        node = self.nodes.get(node_id)
        other = self.nodes.get(toward_id)
        if node is None or other is None:
            return
        want = math.atan2(other.xy[1] - node.xy[1],
                          other.xy[0] - node.xy[0])
        best_o, best_err = None, math.pi
        for o in node.openings:
            if o.status != Opening.UNEXPLORED:
                continue
            err = abs(_wrap(o.bearing - want))
            if err < best_err:
                best_o, best_err = o, err
        # Accept the match only if it is reasonably aligned (< 60°).
        if best_o is not None and best_err < math.radians(60.0):
            best_o.status = Opening.EXPLORED
            best_o.edge_id = edge_id

    def _line_crosses_poison(self, a, b):
        """Sample the straight line a→b and ask the grid if any sample is
        poison. Edge-level poison flag for the eventual mission planner."""
        na, nb = self.nodes.get(a), self.nodes.get(b)
        if na is None or nb is None:
            return False
        ax, ay = na.xy
        bx, by = nb.xy
        steps = max(2, int(math.hypot(bx - ax, by - ay) / 0.10))
        for k in range(steps + 1):
            t = k / steps
            wx = ax + t * (bx - ax)
            wy = ay + t * (by - ay)
            try:
                if self._poison_at(wx, wy):
                    return True
            except Exception:
                pass
        return False

    def _poison_at(self, wx, wy):
        """Poison lookup via the most recent Grid2D captured in observe()."""
        if self._grid is None:
            return False
        return self._grid.is_poison_world(wx, wy)


# ---------------------------------------------------------------------------
# small helpers
# ---------------------------------------------------------------------------
def _wrap(a):
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def _circular_runs(mask):
    """Run-length-encode a circular boolean array.

    Returns a list of ``(start_index, count)`` for every maximal run of
    True values, correctly handling a run that wraps the 0/N boundary.
    """
    n = len(mask)
    if n == 0:
        return []
    if mask.all():
        return [(0, n)]
    if not mask.any():
        return []
    # Start scanning from the first False so a wrap-around run is contiguous.
    start = int(np.argmin(mask))
    runs = []
    run_start = None
    count = 0
    for k in range(n):
        idx = (start + k) % n
        if mask[idx]:
            if run_start is None:
                run_start = idx
                count = 0
            count += 1
        else:
            if run_start is not None:
                runs.append((run_start, count))
                run_start = None
                count = 0
    if run_start is not None:
        runs.append((run_start, count))
    return runs
