"""
vrp_solver.py
=========================================================
Generalized CVRP Solver  (Mission 2)
AI Hackathon: Route Optimization 2026 - AI for Green Logistics
=========================================================

ONE solver for ANY VRPLIB/TSPLIB-style CVRP file
(NAME / TYPE / DIMENSION / EDGE_WEIGHT_TYPE / CAPACITY /
 NODE_COORD_SECTION / DEMAND_SECTION / DEPOT_SECTION / EOF).

Rules we follow:
  - No hard-coded routes or instance-specific logic.
  - Same code path for every instance; only the input file changes.
  - Every solution is checked for feasibility BEFORE its cost/gap is reported.

Algorithm
  1. Construction : Clarke & Wright parallel Savings (capacity-checked)
  2. Improvement  : repeat until no gain
       - Relocate  : move 1 customer to its best position in another route
                     (a route left empty is removed -> fewer vehicles)
       - Swap      : exchange 1 customer between 2 routes
       - 2-opt     : remove crossings inside a single route

Design note (used by Mission 3):
  Every improvement move asks a `route_ok(route)` function whether the new
  route is allowed. By default it only checks capacity (plain CVRP). Mission 3
  passes a stricter function (capacity + freshness time) and reuses the SAME
  moves without copying or editing this file.
"""

from __future__ import annotations

import itertools
import math
import os
import re
from dataclasses import dataclass
from typing import Callable, Dict, List, Optional, Tuple

Route = List[int]
DistMatrix = Dict[Tuple[int, int], int]
RouteCheck = Callable[[Route], bool]


# =========================================================
# 1. DATA STRUCTURES
# =========================================================

@dataclass
class VRPInstance:
    name: str
    dimension: int
    capacity: int
    coords: Dict[int, Tuple[float, float]]
    demands: Dict[int, int]
    depot_id: int
    num_vehicles: Optional[int] = None      # from COMMENT or "-kN" in the name
    optimal_value: Optional[float] = None   # from COMMENT, if published
    source_file: str = ""

    @property
    def customer_ids(self) -> List[int]:
        return [n for n in self.coords if n != self.depot_id]


@dataclass
class SolverResult:
    instance_name: str
    routes: List[Route]
    route_loads: List[int]
    total_distance: int
    feasible: bool
    feasibility_report: List[str]
    optimal_value: Optional[float]
    gap_percent: Optional[float]
    num_vehicles_used: int
    algorithm: str = "Clarke-Wright Savings + Relocate + Swap + 2-opt (capacity-checked)"


# =========================================================
# 2. PARSER
# =========================================================

def parse_vrp_file(filepath: str) -> VRPInstance:
    with open(filepath, "r", encoding="utf-8", errors="ignore") as f:
        lines = [line.strip() for line in f]

    name, dimension, capacity = "", 0, 0
    num_vehicles, optimal_value = None, None
    coords: Dict[int, Tuple[float, float]] = {}
    demands: Dict[int, int] = {}
    depot_id, section = 1, None

    for raw in lines:
        if not raw:
            continue
        upper = raw.upper()
        if ":" in raw and not upper.startswith(("NODE_COORD_SECTION", "DEMAND_SECTION", "DEPOT_SECTION")):
            key, _, value = raw.partition(":")
            key, value = key.strip().upper(), value.strip()
            if key == "NAME":
                name = value
            elif key == "DIMENSION":
                dimension = int(value)
            elif key == "CAPACITY":
                capacity = int(value)
            elif key == "COMMENT":
                m = re.search(r"No of trucks:\s*(\d+)", value, re.I)
                if m:
                    num_vehicles = int(m.group(1))
                m = re.search(r"Optimal value:\s*([\d.]+)", value, re.I)
                if m:
                    optimal_value = float(m.group(1))
            continue

        if upper.startswith("NODE_COORD_SECTION"):
            section = "COORD"; continue
        if upper.startswith("DEMAND_SECTION"):
            section = "DEMAND"; continue
        if upper.startswith("DEPOT_SECTION"):
            section = "DEPOT"; continue
        if upper.startswith("EOF"):
            break

        parts = raw.split()
        if section == "COORD" and len(parts) >= 3:
            coords[int(parts[0])] = (float(parts[1]), float(parts[2]))
        elif section == "DEMAND" and len(parts) >= 2:
            demands[int(parts[0])] = int(float(parts[1]))
        elif section == "DEPOT" and parts:
            val = int(parts[0])
            if val != -1:
                depot_id = val

    if not name:
        name = os.path.splitext(os.path.basename(filepath))[0]
    if num_vehicles is None:                       # naming convention "...-kN"
        m = re.search(r"-k(\d+)", name, re.I)
        if m:
            num_vehicles = int(m.group(1))

    return VRPInstance(name, dimension or len(coords), capacity, coords, demands,
                       depot_id, num_vehicles, optimal_value, filepath)


# =========================================================
# 3. DISTANCE  (TSPLIB EUC_2D = Euclidean rounded to nearest integer)
# =========================================================

def euc_2d_round(p1, p2) -> int:
    return math.floor(math.hypot(p1[0] - p2[0], p1[1] - p2[1]) + 0.5)


def build_distance_matrix(instance: VRPInstance) -> DistMatrix:
    ids = list(instance.coords)
    return {(i, j): 0 if i == j else euc_2d_round(instance.coords[i], instance.coords[j])
            for i in ids for j in ids}


def route_length(route: Route, dist: DistMatrix) -> int:
    return sum(dist[(route[k], route[k + 1])] for k in range(len(route) - 1))


def total_length(routes: List[Route], dist: DistMatrix) -> int:
    return sum(route_length(r, dist) for r in routes)


def route_load(route: Route, instance: VRPInstance) -> int:
    return sum(instance.demands.get(c, 0) for c in route[1:-1])


def capacity_check(instance: VRPInstance) -> RouteCheck:
    """Default feasibility rule for plain CVRP: load <= capacity."""
    return lambda r: route_load(r, instance) <= instance.capacity


# =========================================================
# 4. CONSTRUCTION: CLARKE & WRIGHT SAVINGS
# =========================================================

def clarke_wright_savings(instance: VRPInstance, dist: DistMatrix,
                          route_ok: Optional[RouteCheck] = None) -> List[Route]:
    route_ok = route_ok or capacity_check(instance)
    depot, customers = instance.depot_id, instance.customer_ids

    routes = {c: [depot, c, depot] for c in customers}
    route_of = {c: c for c in customers}

    savings = []
    for a, i in enumerate(customers):
        for j in customers[a + 1:]:
            savings.append((dist[(depot, i)] + dist[(depot, j)] - dist[(i, j)], i, j))
    savings.sort(key=lambda t: t[0], reverse=True)

    for s, i, j in savings:
        if s <= 0:
            continue
        ri, rj = route_of[i], route_of[j]
        if ri == rj:
            continue
        route_i, route_j = routes[ri], routes[rj]
        # i and j must sit next to the depot in their own route
        if i not in (route_i[1], route_i[-2]) or j not in (route_j[1], route_j[-2]):
            continue
        seg_i = route_i[1:-1] if route_i[-2] == i else route_i[1:-1][::-1]
        seg_j = route_j[1:-1] if route_j[1] == j else route_j[1:-1][::-1]
        merged = [depot] + seg_i + seg_j + [depot]
        if not route_ok(merged):
            continue
        routes[ri] = merged
        for c in merged[1:-1]:
            route_of[c] = ri
        del routes[rj]

    return [r for r in routes.values() if len(r) > 2]


# =========================================================
# 5. IMPROVEMENT MOVES (all take a route_ok rule)
# =========================================================

def two_opt(route: Route, dist: DistMatrix, route_ok: Optional[RouteCheck] = None) -> Route:
    """Intra-route 2-opt: reverse a segment if the route gets shorter (and stays allowed)."""
    best, best_len = route[:], route_length(route, dist)
    improved = True
    while improved:
        improved = False
        for a in range(1, len(best) - 2):
            for b in range(a + 1, len(best) - 1):
                cand = best[:a] + best[a:b + 1][::-1] + best[b + 1:]
                cand_len = route_length(cand, dist)
                if cand_len < best_len and (route_ok is None or route_ok(cand)):
                    best, best_len, improved = cand, cand_len, True
    return best


def relocate(routes: List[Route], dist: DistMatrix, route_ok: RouteCheck) -> List[Route]:
    """
    Inter-route relocate: move ONE customer to the best position in another
    route if total distance drops and both routes stay allowed.
    A route may be emptied completely -> it is removed (one truck fewer).
    """
    routes = [r[:] for r in routes]
    improved = True
    while improved:
        improved = False
        for i, r_from in enumerate(routes):
            for pos in range(1, len(r_from) - 1):
                cust = r_from[pos]
                new_from = r_from[:pos] + r_from[pos + 1:]
                gain = route_length(r_from, dist) - (route_length(new_from, dist) if len(new_from) > 2 else 0)
                best = None
                for j, r_to in enumerate(routes):
                    if j == i:
                        continue
                    for k in range(len(r_to) - 1):
                        add = dist[(r_to[k], cust)] + dist[(cust, r_to[k + 1])] - dist[(r_to[k], r_to[k + 1])]
                        delta = gain - add
                        if delta > 0 and (best is None or delta > best[0]):
                            cand = r_to[:k + 1] + [cust] + r_to[k + 1:]
                            if route_ok(cand) and (len(new_from) <= 2 or route_ok(new_from)):
                                best = (delta, j, cand)
                if best:
                    _, j, cand = best
                    routes[j] = cand
                    routes[i] = new_from
                    routes = [r for r in routes if len(r) > 2]
                    improved = True
                    break
            if improved:
                break
    return routes


def swap_customers(routes: List[Route], dist: DistMatrix, route_ok: RouteCheck) -> List[Route]:
    """Inter-route swap: exchange one customer of route A with one of route B."""
    routes = [r[:] for r in routes]
    improved = True
    while improved:
        improved = False
        for i in range(len(routes)):
            for j in range(i + 1, len(routes)):
                r_i, r_j = routes[i], routes[j]
                for pi in range(1, len(r_i) - 1):
                    for pj in range(1, len(r_j) - 1):
                        ci, cj = r_i[pi], r_j[pj]
                        old = (dist[(r_i[pi - 1], ci)] + dist[(ci, r_i[pi + 1])]
                               + dist[(r_j[pj - 1], cj)] + dist[(cj, r_j[pj + 1])])
                        new = (dist[(r_i[pi - 1], cj)] + dist[(cj, r_i[pi + 1])]
                               + dist[(r_j[pj - 1], ci)] + dist[(ci, r_j[pj + 1])])
                        if new >= old:
                            continue
                        ni = r_i[:pi] + [cj] + r_i[pi + 1:]
                        nj = r_j[:pj] + [ci] + r_j[pj + 1:]
                        if route_ok(ni) and route_ok(nj):
                            routes[i], routes[j] = ni, nj
                            r_i, r_j = ni, nj
                            improved = True
    return routes


def improve(routes: List[Route], dist: DistMatrix, route_ok: RouteCheck,
            max_rounds: int = 50) -> List[Route]:
    """Local-search loop: Relocate -> Swap -> 2-opt until nothing improves."""
    best = total_length(routes, dist)
    for _ in range(max_rounds):
        routes = relocate(routes, dist, route_ok)
        routes = swap_customers(routes, dist, route_ok)
        routes = [two_opt(r, dist, route_ok) for r in routes]
        cur = total_length(routes, dist)
        if cur >= best:
            break
        best = cur
    return routes


# =========================================================
# 6. FEASIBILITY CHECK  (mandatory rules)
# =========================================================

def check_feasibility(instance: VRPInstance, routes: List[Route]) -> Tuple[bool, List[str]]:
    report, ok = [], True
    depot, cap = instance.depot_id, instance.capacity

    visited = [c for r in routes for c in r[1:-1]]
    missing = set(instance.customer_ids) - set(visited)
    dup = sorted({c for c in visited if visited.count(c) > 1})

    if missing:
        ok = False; report.append(f"FAIL - Node Coverage: missing {sorted(missing)}")
    else:
        report.append("PASS - Node Coverage: all customers are visited")
    if dup:
        ok = False; report.append(f"FAIL - Visited-once: {dup}")
    else:
        report.append("PASS - Visited-once: no customer appears twice")

    over = [(k + 1, route_load(r, instance)) for k, r in enumerate(routes) if route_load(r, instance) > cap]
    if over:
        ok = False; report.append(f"FAIL - Capacity: over capacity {over}")
    else:
        report.append(f"PASS - Capacity: all routes <= {cap}")

    if instance.num_vehicles is not None and len(routes) > instance.num_vehicles:
        ok = False; report.append(f"FAIL - Fleet Limit: used {len(routes)}, max {instance.num_vehicles}")
    else:
        report.append(f"PASS - Fleet Limit: used {len(routes)}"
                      + (f" / max {instance.num_vehicles}" if instance.num_vehicles else ""))

    bad = [k + 1 for k, r in enumerate(routes) if r[0] != depot or r[-1] != depot]
    if bad:
        ok = False; report.append(f"FAIL - Depot Routing: {bad}")
    else:
        report.append("PASS - Depot Routing: every route starts and ends at the depot")
    return ok, report


def compute_gap(team_cost: float, optimal_cost: Optional[float]) -> Optional[float]:
    if not optimal_cost:
        return None
    return (team_cost - optimal_cost) / optimal_cost * 100.0


# =========================================================
# 7. EXACT REFERENCE FOR SMALL INSTANCES  (set partitioning, n <= ~16)
#    Used only to MEASURE solution quality on self-created cases that have
#    no published optimum. Generic: works for any route_ok rule.
# =========================================================

def exact_small(instance: VRPInstance, dist: DistMatrix, route_ok: Optional[RouteCheck] = None,
                max_vehicles: Optional[int] = None, max_customers: int = 16,
                max_stops_per_route: int = 7):
    """
    Returns (best_total_distance, routes) or None if the instance is too big.
    Step 1: for every subset of customers that fits on one truck, find the
            shortest allowed visiting order (brute force; subsets are small).
    Step 2: pick the set of routes that covers every customer exactly once
            with minimum total distance (DP over bitmasks).
    """
    route_ok = route_ok or capacity_check(instance)
    cust = instance.customer_ids
    n = len(cust)
    if n > max_customers:
        return None
    depot = instance.depot_id
    bit = {c: 1 << k for k, c in enumerate(cust)}

    best_route: Dict[int, Tuple[int, Route]] = {}
    for size in range(1, min(n, max_stops_per_route) + 1):
        for sub in itertools.combinations(cust, size):
            if sum(instance.demands[c] for c in sub) > instance.capacity:
                continue
            found = None
            # try every visiting order (both directions: time rules are direction-dependent)
            for perm in itertools.permutations(sub):
                r = [depot, *perm, depot]
                L = route_length(r, dist)
                if (found is None or L < found[0]) and route_ok(r):
                    found = (L, r)
            if found:
                best_route[sum(bit[c] for c in sub)] = found

    full = (1 << n) - 1
    limit = max_vehicles if max_vehicles is not None else n
    from functools import lru_cache

    masks_by_low = {k: [m for m in best_route if m >> k & 1 and not (m & ((1 << k) - 1))] for k in range(n)}

    @lru_cache(maxsize=None)
    def solve_mask(mask: int, k_left: int):
        if mask == full:
            return (0, ())
        if k_left == 0:
            return (math.inf, ())
        low = next(k for k in range(n) if not mask >> k & 1)
        best = (math.inf, ())
        for m in masks_by_low[low]:
            if m & mask:
                continue
            sub_cost, sub_routes = solve_mask(mask | m, k_left - 1)
            c = sub_cost + best_route[m][0]
            if c < best[0]:
                best = (c, (m,) + sub_routes)
        return best

    cost, ms = solve_mask(0, limit)
    if cost == math.inf:
        return None
    return int(cost), [best_route[m][1] for m in ms]


# =========================================================
# 8. TOP-LEVEL SOLVE  (this is "the one Solver")
# =========================================================

def solve(filepath: str):
    instance = parse_vrp_file(filepath)
    dist = build_distance_matrix(instance)
    ok_rule = capacity_check(instance)

    routes = clarke_wright_savings(instance, dist, ok_rule)
    routes = improve(routes, dist, ok_rule)

    total = total_length(routes, dist)
    loads = [route_load(r, instance) for r in routes]
    feasible, report = check_feasibility(instance, routes)
    gap = compute_gap(total, instance.optimal_value)
    return SolverResult(instance.name, routes, loads, total, feasible, report,
                        instance.optimal_value, gap, len(routes)), instance


def result_card(result: SolverResult) -> str:
    lines = ["=" * 64, f"RESULT CARD - {result.instance_name}", "=" * 64,
             f"Solver / Algorithm : {result.algorithm}", ""]
    for k, (r, load) in enumerate(zip(result.routes, result.route_loads), 1):
        lines.append(f"Route {k} (load={load}): " + " -> ".join(map(str, r)))
    lines += ["", f"Total Distance   : {result.total_distance}"]
    if result.optimal_value is not None:
        lines += [f"Optimal (known)  : {result.optimal_value:g}",
                  f"Gap              : {result.gap_percent:.2f}%"]
    lines += [f"Feasible         : {'YES' if result.feasible else 'NO'}",
              f"Vehicles Used    : {result.num_vehicles_used}", "", "Feasibility checklist:"]
    lines += [f"  - {x}" for x in result.feasibility_report]
    lines.append("=" * 64)
    return "\n".join(lines)


def plot_routes(instance: VRPInstance, routes: List[Route], title: str, ax=None, savepath: str = None):
    """Route map; bbox_inches='tight' so titles/legends are never cut off."""
    import matplotlib.pyplot as plt
    own = ax is None
    if own:
        fig, ax = plt.subplots(figsize=(6.4, 5.6))
    colors = plt.cm.tab10.colors
    for k, r in enumerate(routes):
        xs = [instance.coords[n][0] for n in r]
        ys = [instance.coords[n][1] for n in r]
        ax.plot(xs, ys, "-o", color=colors[k % 10], ms=4, lw=1.6, label=f"Route {k + 1}")
    for n, (x, y) in instance.coords.items():
        if n != instance.depot_id:
            ax.annotate(str(n), (x, y), textcoords="offset points", xytext=(4, 4), fontsize=7)
    dx, dy = instance.coords[instance.depot_id]
    ax.scatter([dx], [dy], marker="*", s=260, c="black", zorder=5, label="Depot")
    ax.set_title(title, fontsize=11)
    ax.set_aspect("equal", adjustable="datalim")
    ax.legend(fontsize=7, loc="upper left", bbox_to_anchor=(1.01, 1.0), frameon=False)
    ax.grid(alpha=0.25)
    if own:
        fig.tight_layout()
        if savepath:
            fig.savefig(savepath, dpi=200, bbox_inches="tight")
        plt.show()
    return ax


if __name__ == "__main__":
    import sys
    files = sys.argv[1:] or sorted(f for f in os.listdir(".") if f.lower().endswith((".txt", ".vrp"))
                                   and "fresh" not in f.lower())
    rows = []
    for fp in files:
        res, inst = solve(fp)
        print(result_card(res), "\n")
        rows.append(res)
    print(f"{'Instance':<12}{'Vehicles':<10}{'Distance':<10}{'Optimal':<10}{'Gap %':<9}Feasible")
    for r in rows:
        print(f"{r.instance_name:<12}{r.num_vehicles_used:<10}{r.total_distance:<10}"
              f"{(r.optimal_value or '-'):<10}{(f'{r.gap_percent:.2f}' if r.gap_percent is not None else '-'):<9}"
              f"{'YES' if r.feasible else 'NO'}")
