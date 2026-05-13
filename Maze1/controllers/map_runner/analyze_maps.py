"""Analyze PLY vs NPZ wall coverage to identify the root cause of missing walls."""
import numpy as np
import math

# ── Load PLY ──
ply_path = "../teleop_mapping/maps/scene.ply"
with open(ply_path) as f:
    lines = f.readlines()
h = next(i for i,l in enumerate(lines) if l.strip()=="end_header")+1
pts = []
for l in lines[h:]:
    p = l.split()
    if len(p)>=3:
        pts.append((float(p[0]),float(p[1]),float(p[2])))
pts = np.array(pts)
print("=== PLY Statistics ===")
print("Total points:", len(pts))
print("X range: [{:.3f}, {:.3f}]".format(pts[:,0].min(), pts[:,0].max()))
print("Y range: [{:.3f}, {:.3f}]".format(pts[:,1].min(), pts[:,1].max()))
print("Z range: [{:.3f}, {:.3f}]".format(pts[:,2].min(), pts[:,2].max()))

z = pts[:,2]
walls = pts[(z>0.03)&(z<0.55)]
floor = pts[z<=0.03]
ceiling = pts[z>=0.55]
print("\nFloor pts (z<=0.03):   ", len(floor))
print("Wall pts  (0.03<z<0.55):", len(walls))
print("Ceiling pts (z>=0.55):", len(ceiling))

# ── Build PLY grid ──
RES=0.04; X_MIN,X_MAX=-3.0,4.0; Y_MIN,Y_MAX=-5.0,3.0
nx_g=int((X_MAX-X_MIN)/RES); ny_g=int((Y_MAX-Y_MIN)/RES)
ply_grid = np.zeros((nx_g,ny_g),dtype=bool)
ix=((walls[:,0]-X_MIN)/RES).astype(int)
iy=((walls[:,1]-Y_MIN)/RES).astype(int)
m=(ix>=0)&(ix<nx_g)&(iy>=0)&(iy<ny_g)
ply_grid[ix[m],iy[m]]=True
print("\nPLY 2D wall cells (raw):", ply_grid.sum())

from scipy.ndimage import binary_dilation
ply_dilated = binary_dilation(ply_grid, iterations=1)
print("PLY 2D wall cells (dilated 1x):", ply_dilated.sum())

# ── Build NPZ grid ──
npz = np.load("maps/map.npz", allow_pickle=False)
src = npz["occupied"].astype(bool)
if "aux_obstacle" in npz.files:
    src = src | npz["aux_obstacle"].astype(bool)
sr = float(npz["resolution"])
sox,soy = float(npz["origin"][0]),float(npz["origin"][1])
npz_grid = np.zeros((nx_g,ny_g),dtype=bool)
ii,jj=np.where(src)
sx=sox+(ii+0.5)*sr; sy=soy+(jj+0.5)*sr
nix=((sx-X_MIN)/RES).astype(int); niy=((sy-Y_MIN)/RES).astype(int)
nm=(nix>=0)&(nix<nx_g)&(niy>=0)&(niy<ny_g)
npz_grid[nix[nm],niy[nm]]=True
print("\nNPZ 2D wall cells:", npz_grid.sum())

# ── Compare coverage ──
both = ply_dilated & npz_grid
ply_only = ply_dilated & ~npz_grid
npz_only = npz_grid & ~ply_dilated
print("\nBoth PLY+NPZ agree:", both.sum())
print("PLY-only (walls NPZ missed):", ply_only.sum())
print("NPZ-only (noise NPZ added):", npz_only.sum())

# ── Check critical areas ──
# Near Yellow pillar
yw_ix_min=int((0.2-X_MIN)/RES); yw_ix_max=int((0.7-X_MIN)/RES)
yw_iy_min=int((-0.9-Y_MIN)/RES); yw_iy_max=int((-0.5-Y_MIN)/RES)
ply_region = ply_dilated[yw_ix_min:yw_ix_max, yw_iy_min:yw_iy_max].sum()
npz_region = npz_grid[yw_ix_min:yw_ix_max, yw_iy_min:yw_iy_max].sum()
print("\nNear Yellow (x:0.2-0.7, y:-0.9--0.5):")
print("  PLY wall cells:", ply_region)
print("  NPZ wall cells:", npz_region)

# Center of maze (where the path cut through)
center_ix_min=int((0.5-X_MIN)/RES); center_ix_max=int((2.0-X_MIN)/RES)
center_iy_min=int((-0.5-Y_MIN)/RES); center_iy_max=int((0.5-Y_MIN)/RES)
ply_center = ply_dilated[center_ix_min:center_ix_max, center_iy_min:center_iy_max].sum()
npz_center = npz_grid[center_ix_min:center_ix_max, center_iy_min:center_iy_max].sum()
print("\nCenter of maze (x:0.5-2.0, y:-0.5-0.5):")
print("  PLY wall cells:", ply_center)
print("  NPZ wall cells:", npz_center)

# NPZ metadata
res_val = float(npz["resolution"])
ox_val = float(npz["origin"][0])
oy_val = float(npz["origin"][1])
cells_val = int(npz["cells"])
bx_val = float(npz["pillar_blue"][0])
by_val = float(npz["pillar_blue"][1])
yx_val = float(npz["pillar_yellow"][0])
yy_val = float(npz["pillar_yellow"][1])
print("\nNPZ metadata:")
print("  resolution: {:.4f}".format(res_val))
print("  origin:     ({:.3f}, {:.3f})".format(ox_val, oy_val))
print("  cells:      {}".format(cells_val))
print("  pillar_blue:   ({:.3f}, {:.3f})".format(bx_val, by_val))
print("  pillar_yellow: ({:.3f}, {:.3f})".format(yx_val, yy_val))

# ── Generate comparison visualization ──
try:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from scipy.ndimage import distance_transform_edt
    
    fig, axes = plt.subplots(2, 3, figsize=(24, 16))
    fig.suptitle("PLY vs NPZ Wall Map Comparison — Root Cause of A* Failure", fontsize=16, fontweight='bold')
    
    # 1. NPZ wall map (what A* currently uses)
    ax = axes[0, 0]
    wi, wj = np.where(npz_grid)
    wx_w = X_MIN + (wi + 0.5) * RES
    wy_w = Y_MIN + (wj + 0.5) * RES
    ax.scatter(wx_w, wy_w, s=0.5, c='black', alpha=0.5)
    ax.plot(bx_val, by_val, 'o', color='blue', ms=12)
    ax.plot(yx_val, yy_val, 'o', color='gold', ms=12)
    ax.plot(0, 0, '^', color='green', ms=12)
    ax.set_title("A) NPZ Walls (map.npz['occupied'])\n— WHAT A* USES NOW —", fontsize=11)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_aspect('equal'); ax.grid(True, alpha=0.2)
    ax.set_xlim(-0.5, 3.2); ax.set_ylim(-1.6, 2.0)
    
    # 2. PLY wall map (what it should use)
    ax = axes[0, 1]
    wi2, wj2 = np.where(ply_dilated)
    wx2 = X_MIN + (wi2 + 0.5) * RES
    wy2 = Y_MIN + (wj2 + 0.5) * RES
    ax.scatter(wx2, wy2, s=0.5, c='black', alpha=0.5)
    ax.plot(bx_val, by_val, 'o', color='blue', ms=12)
    ax.plot(yx_val, yy_val, 'o', color='gold', ms=12)
    ax.plot(0, 0, '^', color='green', ms=12)
    ax.set_title("B) PLY Walls (scene.ply projected)\n— WHAT A* SHOULD USE —", fontsize=11)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_aspect('equal'); ax.grid(True, alpha=0.2)
    ax.set_xlim(-0.5, 3.2); ax.set_ylim(-1.6, 2.0)
    
    # 3. Difference map
    ax = axes[0, 2]
    ply_only_i, ply_only_j = np.where(ply_only)
    npz_only_i, npz_only_j = np.where(npz_only)
    both_i, both_j = np.where(both)
    if len(both_i) > 0:
        ax.scatter(X_MIN+(both_i+0.5)*RES, Y_MIN+(both_j+0.5)*RES, s=0.3, c='grey', alpha=0.3, label='Both agree')
    if len(ply_only_i) > 0:
        ax.scatter(X_MIN+(ply_only_i+0.5)*RES, Y_MIN+(ply_only_j+0.5)*RES, s=1, c='red', alpha=0.7, label='PLY-only (NPZ MISSED)')
    if len(npz_only_i) > 0:
        ax.scatter(X_MIN+(npz_only_i+0.5)*RES, Y_MIN+(npz_only_j+0.5)*RES, s=1, c='blue', alpha=0.5, label='NPZ-only (extra)')
    ax.plot(bx_val, by_val, 'o', color='blue', ms=12)
    ax.plot(yx_val, yy_val, 'o', color='gold', ms=12)
    ax.plot(0, 0, '^', color='green', ms=12)
    ax.set_title("C) Wall Difference Map\nRED = walls NPZ missed (CRITICAL)", fontsize=11)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.legend(fontsize=8, loc='upper left')
    ax.set_aspect('equal'); ax.grid(True, alpha=0.2)
    ax.set_xlim(-0.5, 3.2); ax.set_ylim(-1.6, 2.0)
    
    # 4. PLY EDT (distance to walls)
    ax = axes[1, 0]
    edt = distance_transform_edt(~ply_dilated) * RES
    edt_plot = edt.T  # Transpose for correct orientation
    im = ax.imshow(edt_plot, origin='lower', 
                   extent=[X_MIN, X_MAX, Y_MIN, Y_MAX],
                   cmap='RdYlGn', vmin=0, vmax=0.5, aspect='equal')
    ax.set_title("D) EDT Distance to Walls (PLY)\nGreen=safe, Red=danger", fontsize=11)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_xlim(-0.5, 3.2); ax.set_ylim(-1.6, 2.0)
    plt.colorbar(im, ax=ax, label='Distance to wall [m]', shrink=0.7)
    
    # 5. Inflated wall (INFLATE=3)
    ax = axes[1, 1]
    inflated = binary_dilation(ply_dilated, iterations=3)
    inf_i, inf_j = np.where(inflated)
    ax.scatter(X_MIN+(inf_i+0.5)*RES, Y_MIN+(inf_j+0.5)*RES, s=0.3, c='red', alpha=0.15, label='Inflated (12cm)')
    raw_i, raw_j = np.where(ply_dilated)
    ax.scatter(X_MIN+(raw_i+0.5)*RES, Y_MIN+(raw_j+0.5)*RES, s=0.3, c='black', alpha=0.5, label='Raw wall')
    ax.plot(bx_val, by_val, 'o', color='blue', ms=12)
    ax.plot(yx_val, yy_val, 'o', color='gold', ms=12)
    ax.plot(0, 0, '^', color='green', ms=12)
    ax.set_title("E) PLY Walls + Inflation (r=3, 12cm)\nRed=robot-unsafe zone", fontsize=11)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.legend(fontsize=8, loc='upper left')
    ax.set_aspect('equal'); ax.grid(True, alpha=0.2)
    ax.set_xlim(-0.5, 3.2); ax.set_ylim(-1.6, 2.0)
    
    # 6. A* cost map
    ax = axes[1, 2]
    cost = np.ones_like(edt)
    cost[inflated] = np.inf
    mask = (~inflated) & (edt < 12*RES)  # WALL_PROX_CELLS=12
    cost[mask] += 10.0 * (1.0 - edt[mask] / (12*RES))  # WALL_PROX_WEIGHT=10
    cost_vis = cost.copy()
    cost_vis[np.isinf(cost_vis)] = 20
    im2 = ax.imshow(cost_vis.T, origin='lower',
                    extent=[X_MIN, X_MAX, Y_MIN, Y_MAX],
                    cmap='hot_r', vmin=1, vmax=12, aspect='equal')
    ax.set_title("F) A* Cost Map (PLY-based)\nDark=cheap, Light=expensive", fontsize=11)
    ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
    ax.set_xlim(-0.5, 3.2); ax.set_ylim(-1.6, 2.0)
    plt.colorbar(im2, ax=ax, label='A* traversal cost', shrink=0.7)
    
    fig.tight_layout()
    fig.savefig("maps/ply_vs_npz_comparison.png", dpi=150)
    plt.close(fig)
    print("\n[viz] saved maps/ply_vs_npz_comparison.png")
except Exception as e:
    print("Viz error:", e)
    import traceback; traceback.print_exc()
