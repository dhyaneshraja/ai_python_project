"""
Dijkstra-Based Energy-Aware Routing in IoT (simple version)
Only uses Python's standard library (heapq).

Idea: normal Dijkstra picks the shortest path. Here the "cost" of a link
also depends on how much battery the receiving node has left, so the
route avoids nodes that are running low and spreads the load.
"""
import heapq

# ---------- 1. Network: node -> {neighbour: distance in metres} ----------
graph = {
    "Sensor": {"A": 10, "B": 14},
    "A":      {"Sensor": 10, "C": 12, "D": 20},
    "B":      {"Sensor": 14, "D": 12},
    "C":      {"A": 12, "Sink": 16},
    "D":      {"A": 20, "B": 12, "Sink": 12},
    "Sink":   {"C": 16, "D": 12},
}

# ---------- 2. Residual energy of each node (Joules) ----------
INITIAL_ENERGY = 5.0
energy = {n: INITIAL_ENERGY for n in graph}
energy["Sink"] = float("inf")          # sink is mains powered

# Radio energy model (simplified): E_tx = E_ELEC + EPS * d^2 per packet
E_ELEC = 0.05    # J per packet (electronics)
EPS = 0.0005     # J per packet per m^2 (amplifier)


def tx_energy(d):
    """Energy needed to send one packet over distance d."""
    return E_ELEC + EPS * d ** 2


def link_cost(u, v, d, energy_aware=True):
    """Cost of hopping u -> v."""
    cost = tx_energy(d)
    if energy_aware and energy[v] != float("inf"):
        # the lower the receiver's battery, the more expensive the link
        cost = cost * (INITIAL_ENERGY / energy[v])
    return cost


# ---------- 3. Dijkstra ----------
def dijkstra(src, dst, energy_aware=True):
    dist = {n: float("inf") for n in graph}
    prev = {}
    dist[src] = 0
    pq = [(0, src)]                     # (cost so far, node)

    while pq:
        cost, u = heapq.heappop(pq)
        if u == dst:
            break
        if cost > dist[u]:
            continue
        for v, d in graph[u].items():
            if energy[v] <= 0:          # dead node, skip
                continue
            new_cost = cost + link_cost(u, v, d, energy_aware)
            if new_cost < dist[v]:
                dist[v] = new_cost
                prev[v] = u
                heapq.heappush(pq, (new_cost, v))

    # rebuild path
    path, node = [], dst
    while node != src:
        path.append(node)
        node = prev[node]
    path.append(src)
    return path[::-1]


# ---------- 4. Send packets and drain batteries ----------
def send_packet(path):
    for u, v in zip(path, path[1:]):
        energy[u] -= tx_energy(graph[u][v])


def simulate(packets, energy_aware):
    for n in graph:
        energy[n] = INITIAL_ENERGY
    energy["Sink"] = float("inf")

    last_path = None
    for i in range(1, packets + 1):
        try:
            path = dijkstra("Sensor", "Sink", energy_aware)
        except KeyError:                # no route left
            print(f"  Packet {i}: network failed (no route)")
            return i - 1
        if path != last_path:
            print(f"  Packet {i:3d}: route = {' -> '.join(path)}")
            last_path = path
        send_packet(path)
        if min(e for n, e in energy.items() if n != "Sink") <= 0:
            print(f"  First node died after packet {i}")
            return i
    return packets


if __name__ == "__main__":
    print("=== Plain shortest-path Dijkstra ===")
    life1 = simulate(200, energy_aware=False)

    print("\n=== Energy-aware Dijkstra ===")
    life2 = simulate(200, energy_aware=True)

    print("\n--- Result ---")
    print(f"Network lifetime (plain Dijkstra)  : {life1} packets")
    print(f"Network lifetime (energy-aware)    : {life2} packets")
    print("\nRemaining energy (energy-aware run):")
    for n, e in energy.items():
        if n != "Sink":
            print(f"  {n:6s}: {e:.2f} J")
